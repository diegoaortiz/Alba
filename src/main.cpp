/*
 * CO2 Display with WiFi Captive-Portal Provisioning
 * Pico 2W + MH-Z19B + GC9A01 Round LCD
 *
 * Rendering: LVGL v9 (declarative, anti-aliased, animation-engine driven)
 *   - Arduino_GFX is used only as the low-level flush driver for the GC9A01.
 *   - A "target" of concentric filled circles (Andre Lemonnier style) breathes:
 *     the whole stack scales in/out from a shared cosine oscillator, with a
 *     gentle per-ring phase wave so the rings ripple.
 *   - The ring colours form a gradient (bright core -> rim) sampled from the
 *     CO2 level; the breathing rate is also driven by the CO2 level.
 *
 * WiFi provisioning: open Access-Point + captive portal (inspired by
 *   ModuleAir_V4), credentials stored in flash (LittleFS).
 *
 * Control flow is cooperative: a single non-blocking loop drives LVGL,
 * the web server, the CO2 sensor and the LED ring together.
 */

#include <Arduino.h>
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <WS2812FX.h> // pulls in Adafruit_NeoPixel as its driver
#include <SerialPIO.h>
#include <math.h>

// WiFi + captive portal + storage
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <HTTPClient.h>       // POST measurements to the AirCarto endpoint
#include <WiFiClientSecure.h> // TLS for the https:// data server

// ============================================================
// PIN CONFIGURATION - Pico 2W
// ============================================================

#define TFT_SCK 10
#define TFT_MOSI 11
#define TFT_CS 13
#define TFT_DC 14
#define TFT_RST 15
#define TFT_BL 12

#define LED_PIN 22
#define LED_COUNT 64
#define LED_BRIGHTNESS 90

// MH-Z19 on GP6 / GP7. These pins are NOT on a hardware UART, so we drive a
// PIO-emulated serial port (SerialPIO). GP6 = TX, GP7 = RX.
// Wire crossed: sensor TX -> GP7 (Pico RX), sensor RX -> GP6 (Pico TX).
#define MHZ_TX 6
#define MHZ_RX 7
SerialPIO mhzSerial(MHZ_TX, MHZ_RX);

// ============================================================
// DISPLAY (Arduino_GFX = LVGL flush backend)
// ============================================================

#define SCREEN_W 240
#define SCREEN_H 240

Arduino_DataBus *bus = new Arduino_RPiPicoSPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI,
                                              -1 /* MISO */, spi1);
Arduino_GC9A01 *gfx =
    new Arduino_GC9A01(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

// LED ring driven by WS2812FX (built-in BREATH effect = the soft "standby"
// heartbeat). Its driver is Adafruit_NeoPixel, the same path that already
// works on this board. Colour is set from the CO2 level; the breath cadence is
// fixed (slow & gentle) so the screen carries the urgency via its pulse speed.
WS2812FX ws2812fx(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
bool ledRingReady = false;

// Full-screen LVGL draw buffer (RENDER_MODE_FULL): LVGL composes the whole
// frame, then we push it in one go. This avoids the visible "section by
// section" band refresh of partial rendering. Must be aligned (LVGL asserts).
static lv_display_t *lvDisplay = nullptr;
static uint8_t lvDrawBuf[SCREEN_W * SCREEN_H * (LV_COLOR_DEPTH / 8)]
    __attribute__((aligned(64)));

// ============================================================
// ANIMATION CONFIGURATION
// ============================================================

// Concentric "target" of breathing rings (inspired by Andre Lemonnier's
// circles), drawn as ONE full-screen radial gradient whose colour stops form
// distinct bands: bright core colour -> rim colour, then a soft fade to black.
//
// Why a gradient and not stacked circle objects? Breathing has to be smooth.
// Moving hard-edged geometry in whole pixels looks stepped ("par paliers")
// because the motion is slow (the radius only changes ~1px every 100ms), and
// transform_scale (the sub-pixel alternative) needs a 250KB ARGB layer that
// does not fit in RAM. A radial gradient sidesteps both: animating its radius
// re-rasterises softly every frame, so the breath is fluid -- exactly the
// mechanism the original single-colour glow used. The banding (sharp colour
// holds + thin ramps) is what turns that smooth glow into visible rings.
#define RING_CX 120          // centre (screen is 240x240)
#define RING_CY 120
#define RING_COUNT 8         // number of colour bands
#define RING_RAMP 4          // soft transition width between bands (1/255 units)
#define RING_GRAD_RMAX 150   // gradient radius (px) at full breath (frac 255)
#define RING_SCALE_MIN 0.60f // trough scale (whole target shrinks to 60%)
#define RING_FRAME_MS 16     // ~60 fps (single gradient fill is cheap)

// ============================================================
// CONFIGURATION
// ============================================================

#define WIFI_CONFIG_FILE "/wifi.cfg"

// --- Feature flags ---------------------------------------------------------
// SKIP_WIFI=1 bypasses the whole WiFi/captive-portal flow and jumps straight
// to the live CO2 display. Needed on a plain Pico 2 (no radio) and handy while
// working on the display/animation. Set to 0 to re-enable provisioning.
// Re-enabled: full ModuleAir_V4 connection state machine (connect -> scan ->
// retry -> AP fallback -> periodic background reconnect).
#define SKIP_WIFI 0
// CO2_DEBUG=1 prints the raw 9-byte MH-Z19 UART frames so we can tell a wiring
// problem (no bytes / wrong header) from a warm-up (out-of-range value) issue.
#define CO2_DEBUG 1
// How long the "alba" splash screen stays before switching to the live display.
#define SPLASH_MS 4500

// Captive-portal network configuration (mirrors ModuleAir_V4)
#define AP_IP_OCT_1 192
#define AP_IP_OCT_2 168
#define AP_IP_OCT_3 4
#define AP_IP_OCT_4 1
#define DNS_PORT 53
#define WEB_PORT 80

// --- Data upload (AirCarto Alba ingestion endpoint) ------------------------
// JSON POST body, format defined by api.aircarto.fr/capteurs/alba.php.
#define DATA_SERVER_URL "https://api.aircarto.fr/capteurs/alba.php"
#define DATA_SEND_INTERVAL 60000UL // push a measurement every 60 s (ModuleAir)
#define DATA_WARMUP_MS 15000UL     // first push ~15 s after (re)connect
#define FIRMWARE_VERSION "1.0.0"   // reported as version_major.minor.patch
#define PROTOCOL_VERSION 1         // "version" field expected by the endpoint

// --- WiFi connection state-machine timings (mirrors ModuleAir_V4) ----------
#define WIFI_CONNECT_TIMEOUT_MS 15000UL       // one STA connection attempt
#define WIFI_MAX_ATTEMPTS 2                   // boot tries before AP fallback
#define AP_CONFIG_DURATION_MS (3UL * 60 * 1000)  // "Config WiFi" splash, then ppm
#define AP_RETRY_INTERVAL_MS (10UL * 60 * 1000)  // background reconnect cadence
#define AP_RETRY_TIMEOUT_MS (30UL * 1000)        // one background reconnect try
#define STA_RECONNECT_WINDOW_MS (3UL * 60 * 1000) // recover a dropped STA link
#define STA_RECONNECT_KICK_MS (30UL * 1000)       // re-kick driver while recovering

// Device modes (drive the DISPLAY: splash / config screen / ppm)
enum DeviceMode { MODE_STARTUP, MODE_CONFIG, MODE_NORMAL };

// WiFi connection FSM (drives the RADIO; independent of the display mode above).
// Mirrors ModuleAir_V4's wifi_manager state machine. Concurrent AP+STA is used
// for the background reconnect (WIFI_AP_STA) so the config hotspot stays up the
// whole time -- reliable since arduino-pico 5.5.1 (PR #3374), which we pin.
enum WifiState {
  WS_STA_CONNECTED,    // connected to a known network, normal operation
  WS_STA_RECONNECTING, // link dropped: trying to recover for up to 3 min
  WS_AP_CONFIG,        // AP up, screen shows "Config WiFi" (first 3 min)
  WS_AP_DATA,          // AP still up but screen shows ppm; retries every 10 min
  WS_AP_RETRYING,      // AP+STA: attempting a background reconnect (AP stays up)
};

// ============================================================
// GLOBALS
// ============================================================

uint16_t co2Value = 0; // last instantaneous reading (drives the live display)

// Averaging accumulators for the value we UPLOAD. Like ModuleAir-Next-Gen, we
// average every valid CO2 reading taken during the send window and transmit the
// mean (not the last instantaneous value), then reset the window on each send.
uint32_t co2Sum = 0;
uint16_t co2SampleCount = 0;

unsigned long lastCO2Request = 0;

DeviceMode currentMode = MODE_STARTUP;

// WiFi connection state machine
WifiState wifiState = WS_AP_CONFIG;       // set for real once we leave startup
unsigned long stateEnteredAt = 0;         // millis() when wifiState last changed
unsigned long lastReconnectKickAt = 0;    // last WiFi.reconnect() during recovery
bool serverStarted = false;               // HTTP server begun at least once
bool routesRegistered = false;            // HTTP routes registered (once only)

// WiFi credentials
String storedSSID = "";
String storedPassword = "";

// AP / captive portal state
String chipId = "";
String apSSID = "";
WebServer server(WEB_PORT);
DNSServer dnsServer;
String cachedScanJson = "[]"; // networks scanned once before the AP comes up

// Device identity (derived from the WiFi MAC in setup()).
//   deviceToken : printable serial, MAC-derived, also shown in the SSID
//                 ("alba-<token>"); this is what gets registered server-side.
//   deviceIdHex : hex encoding of deviceToken's ASCII bytes -- the "device_id"
//                 field the endpoint expects (it does pack('H*') to recover the
//                 token). e.g. token "A1B2C3" -> device_id "413142324333".
String deviceToken = "";
String deviceIdHex = "";
String deviceMac = "n/a";       // colon-formatted MAC, for the identity banner
bool tokenFromChipId = false;   // true if the MAC was unavailable (fallback)
unsigned long lastDataSend = 0; // millis() of the last upload attempt

// Current pulse theme (driven by CO2 level). currentDurMs is the breathing
// period, used by breathLevel() which both the glow and the LED ring sample.
static uint16_t currentDurMs = 6000;
static int currentLevel = -1; // -1 forces first theme apply

const uint8_t CO2_CMD[] = {0xFF, 0x01, 0x86, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x79};

// LVGL UI objects
static lv_obj_t *glow;          // full-screen object carrying the banded gradient
static lv_style_t glowStyle;
static lv_grad_dsc_t glowGrad;  // referenced by glowStyle; modified in place
static lv_obj_t *lblTitle;
static lv_obj_t *lblValue;
static lv_obj_t *lblSub;
static lv_obj_t *lblIcon; // big WiFi / OK / X glyph for the connection screens

// Forward declarations
void finishStartup();
void onCO2Updated();
void setWifiState(WifiState s);
void startWebServer();
void apStart();
void showNormalDisplay();
void enterStaConnected();
void startConfigMode();
void attemptBackgroundReconnect();
bool wifiSsidIsVisible(const String &ssid);
bool isApActive();
void wifiManagerLoop();
void dataSenderSend();
void printIdentity();

// ============================================================
// CO2 -> COLOR / SPEED MAPPING
// ============================================================

void getCO2Colors(uint16_t co2, uint8_t *startRGB, uint8_t *endRGB) {
  if (co2 < 600) {
    // Vivid green core -> deep teal rim. The old green->cyan pair both had
    // G=255, so the 9 rings barely differed and melted into one mass; this
    // wider swing keeps the rings distinct (like the other levels do).
    startRGB[0] = 120; startRGB[1] = 255; startRGB[2] = 80;
    endRGB[0] = 0;     endRGB[1] = 120;   endRGB[2] = 160;
  } else if (co2 < 800) {
    startRGB[0] = 0;   startRGB[1] = 255; startRGB[2] = 255;
    endRGB[0] = 0;     endRGB[1] = 0;     endRGB[2] = 200;
  } else if (co2 < 1200) {
    startRGB[0] = 0;   startRGB[1] = 0;   startRGB[2] = 200;
    endRGB[0] = 255;   endRGB[1] = 0;     endRGB[2] = 255;
  } else {
    startRGB[0] = 255; startRGB[1] = 0;   startRGB[2] = 0;
    endRGB[0] = 255;   endRGB[1] = 100;   endRGB[2] = 0;
  }
}

int co2Level(uint16_t co2) {
  if (co2 < 600) return 0;
  if (co2 < 800) return 1;
  if (co2 < 1200) return 2;
  return 3;
}

uint16_t getRingDuration(uint16_t co2) {
  switch (co2Level(co2)) {
    case 0: return 7000; // calm, very slow breath
    case 1: return 6000;
    case 2: return 5000;
    default: return 4000; // worst air: a bit quicker, still gentle
  }
}

// ============================================================
// LVGL DISPLAY DRIVER
// ============================================================

static uint32_t lvMillisCb(void) { return millis(); }

static void lvLogCb(lv_log_level_t level, const char *buf) {
  (void)level;
  Serial.print(buf);
}

static void lvFlushCb(lv_display_t *disp, const lv_area_t *area,
                      uint8_t *px_map) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
  lv_display_flush_ready(disp);
}

// ============================================================
// LVGL ANIMATION CALLBACKS
// ============================================================

// THE breathing oscillator: a single stateless function of time so everything
// that samples it pulses in sync by construction (no shared mutable state).
// 0 -> 1 -> 0 (soft cosine) over currentDurMs; the rate is currentDurMs.
float breathLevel() {
  float t = (float)(millis() % currentDurMs) / (float)currentDurMs;
  return 0.5f - 0.5f * cosf(t * 2.0f * (float)M_PI);
}

// Linear RGB interpolation between two colours (t = 0 -> a, t = 1 -> b).
static lv_color_t lerpColor(const uint8_t *a, const uint8_t *b, float t) {
  return lv_color_make((uint8_t)(a[0] + (b[0] - a[0]) * t),
                       (uint8_t)(a[1] + (b[1] - a[1]) * t),
                       (uint8_t)(a[2] + (b[2] - a[2]) * t));
}

// (Re)build the gradient stops so the colour goes from the bright "start"
// colour at the core (frac 0) to the "end" colour at the rim, split into
// RING_COUNT solid bands. Each band holds its colour, then a thin RING_RAMP
// transition to the next -> distinct rings (not one smooth blend) while the
// soft ramps keep the breathing fluid. The outermost band fades to transparent
// so the whole target sits on black and the breath reads against it.
static void recolorRings(const uint8_t *startRGB, const uint8_t *endRGB) {
  int n = 0;
  for (int k = 0; k < RING_COUNT; k++) {
    float ct = (RING_COUNT > 1) ? (float)k / (RING_COUNT - 1) : 0.0f;
    lv_color_t c = lerpColor(startRGB, endRGB, ct); // core=start, rim=end
    int f0 = k * 255 / RING_COUNT;                  // band start
    int f1 = (k < RING_COUNT - 1) ? ((k + 1) * 255 / RING_COUNT - RING_RAMP)
                                  : 255;            // hold end (last fades out)
    if (f1 < f0) f1 = f0;
    glowGrad.stops[n].color = c;
    glowGrad.stops[n].opa = LV_OPA_COVER;
    glowGrad.stops[n].frac = (uint8_t)f0;
    n++;
    glowGrad.stops[n].color = c;
    glowGrad.stops[n].opa =
        (k == RING_COUNT - 1) ? LV_OPA_TRANSP : LV_OPA_COVER;
    glowGrad.stops[n].frac = (uint8_t)f1;
    n++;
  }
  glowGrad.stops_count = (uint8_t)n;
  if (glow) lv_obj_invalidate(glow);
}

// Set the gradient radius (px) by moving the end circle of the radial gradient,
// then invalidate. Animating this is what makes the banded target breathe --
// the soft re-rasterisation keeps it fluid (no pixel stepping like geometry).
static void setGlowRadius(int32_t r) {
  glowGrad.params.radial.end.x = RING_CX;
  glowGrad.params.radial.end.y = RING_CY;
  glowGrad.params.radial.end_extent.x = RING_CX + r;
  glowGrad.params.radial.end_extent.y = RING_CY;
  if (glow) lv_obj_invalidate(glow);
}

// LVGL timer (native scheduler): sample the oscillator and breathe the radius.
static void glowBreathCb(lv_timer_t *timer) {
  (void)timer;
  float scale = RING_SCALE_MIN + (1.0f - RING_SCALE_MIN) * breathLevel();
  setGlowRadius((int32_t)(RING_GRAD_RMAX * scale));
}

// ============================================================
// LVGL UI
// ============================================================

void buildUI() {
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  // One full-screen object carrying the banded radial gradient. recolorRings()
  // fills the stops; setGlowRadius() (driven by the breath timer) animates the
  // gradient radius so the bands breathe. lv_grad_radial_init() sets the radial
  // direction + extend mode; the focal at radius 0 makes concentric circles.
  uint8_t s0[3] = {0, 200, 255}, e0[3] = {0, 0, 200}; // initial cyan->blue
  lv_grad_radial_init(&glowGrad, RING_CX, RING_CY, RING_CX + RING_GRAD_RMAX,
                      RING_CY, LV_GRAD_EXTEND_PAD);
  lv_grad_radial_set_focal(&glowGrad, RING_CX, RING_CY, 0);

  lv_style_init(&glowStyle);
  lv_style_set_bg_grad(&glowStyle, &glowGrad);
  lv_style_set_bg_opa(&glowStyle, LV_OPA_COVER);
  lv_style_set_border_width(&glowStyle, 0);
  lv_style_set_radius(&glowStyle, 0);
  lv_style_set_pad_all(&glowStyle, 0);

  // Create the object BEFORE filling the stops: recolorRings() invalidates
  // `glow`, so it must already exist (invalidating a NULL object hard-faults
  // the MCU and freezes setup() -- which is exactly what happened).
  glow = lv_obj_create(scr);
  lv_obj_remove_style_all(glow);
  lv_obj_add_style(glow, &glowStyle, 0);
  lv_obj_set_size(glow, SCREEN_W, SCREEN_H);
  lv_obj_center(glow);
  lv_obj_remove_flag(
      glow, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));

  recolorRings(s0, e0); // fill the band stops (also invalidates glow)

  // Labels (created after the gradient so they render on top)
  lblTitle = lv_label_create(scr);
  lv_obj_set_style_text_color(lblTitle, lv_color_white(), 0);
  lv_obj_set_style_text_font(lblTitle, &lv_font_montserrat_14, 0);
  lv_label_set_text(lblTitle, "");
  lv_obj_align(lblTitle, LV_ALIGN_CENTER, 0, -52);

  lblValue = lv_label_create(scr);
  lv_obj_set_style_text_color(lblValue, lv_color_white(), 0);
  lv_obj_set_style_text_font(lblValue, &lv_font_montserrat_48, 0);
  lv_label_set_text(lblValue, "");
  lv_obj_align(lblValue, LV_ALIGN_CENTER, 0, 0);

  lblSub = lv_label_create(scr);
  lv_obj_set_style_text_color(lblSub, lv_color_white(), 0);
  lv_obj_set_style_text_font(lblSub, &lv_font_montserrat_14, 0);
  lv_label_set_text(lblSub, "");
  lv_obj_align(lblSub, LV_ALIGN_CENTER, 0, 50);

  // Big glyph used by the WiFi connection screens (hidden the rest of the time).
  lblIcon = lv_label_create(scr);
  lv_obj_set_style_text_color(lblIcon, lv_color_white(), 0);
  lv_obj_set_style_text_font(lblIcon, &lv_font_montserrat_48, 0);
  lv_label_set_text(lblIcon, "");
  lv_obj_align(lblIcon, LV_ALIGN_CENTER, 0, -46);

  // Drive the ring breathing from the shared oscillator via LVGL's scheduler.
  lv_timer_create(glowBreathCb, RING_FRAME_MS, NULL);
}

// Apply a CO2 theme: recolour the ring gradient on screen + LED breath colour +
// breathing rate. start/end are the CO2 indicator colours (core -> rim).
void applyTheme(const uint8_t *startRGB, const uint8_t *endRGB,
                uint16_t durMs) {
  recolorRings(startRGB, endRGB);
  // LED ring breathes from off to the dominant CO2 colour.
  ws2812fx.setColor(ws2812fx.Color(startRGB[0], startRGB[1], startRGB[2]));
  currentDurMs = durMs; // breathLevel() picks up the new rate immediately
}

// Animation setters used by the splash intro (LVGL animation engine).
static void labelOpaAnimCb(void *obj, int32_t v) {
  lv_obj_set_style_opa((lv_obj_t *)obj, (lv_opa_t)v, 0);
}
static void labelScaleAnimCb(void *obj, int32_t v) {
  lv_obj_set_style_transform_scale((lv_obj_t *)obj, v, 0); // 256 = 1.0x
}

void setStartupUI() {
  uint8_t s[3] = {0, 255, 255};
  uint8_t e[3] = {0, 0, 200};
  applyTheme(s, e, 4000);

  lv_obj_remove_flag(glow, LV_OBJ_FLAG_HIDDEN); // breathing rings back on
  lv_anim_delete(lblIcon, NULL); // stop any leftover icon animation
  lv_label_set_text(lblIcon, "");
  lv_label_set_text(lblTitle, "");
  lv_label_set_text(lblSub, "");

  lv_obj_set_style_text_font(lblValue, &lv_font_montserrat_48, 0);
  lv_label_set_text(lblValue, "alba");
  lv_obj_align(lblValue, LV_ALIGN_CENTER, 0, 0);

  // Intro: "alba" materialises -- fade in + a gentle scale "pop". Scale around
  // the label centre so it grows in place.
  lv_obj_set_style_transform_pivot_x(lblValue, lv_pct(50), 0);
  lv_obj_set_style_transform_pivot_y(lblValue, lv_pct(50), 0);
  lv_obj_set_style_opa(lblValue, LV_OPA_TRANSP, 0);
  lv_obj_set_style_transform_scale(lblValue, 150, 0); // start ~0.6x

  lv_anim_t fade;
  lv_anim_init(&fade);
  lv_anim_set_var(&fade, lblValue);
  lv_anim_set_exec_cb(&fade, labelOpaAnimCb);
  lv_anim_set_values(&fade, LV_OPA_TRANSP, LV_OPA_COVER);
  lv_anim_set_duration(&fade, 900);
  lv_anim_set_path_cb(&fade, lv_anim_path_ease_out);
  lv_anim_start(&fade);

  lv_anim_t pop;
  lv_anim_init(&pop);
  lv_anim_set_var(&pop, lblValue);
  lv_anim_set_exec_cb(&pop, labelScaleAnimCb);
  lv_anim_set_values(&pop, 150, 256);
  lv_anim_set_duration(&pop, 1100);
  lv_anim_set_path_cb(&pop, lv_anim_path_overshoot); // subtle bounce
  lv_anim_start(&pop);
}

void setConfigUI() {
  uint8_t s[3] = {128, 0, 255};
  uint8_t e[3] = {255, 0, 128};
  applyTheme(s, e, 4500);

  lv_obj_remove_flag(glow, LV_OBJ_FLAG_HIDDEN); // breathing rings back on
  // Small WiFi glyph crowning the config screen.
  lv_anim_delete(lblIcon, NULL);
  lv_obj_set_style_opa(lblIcon, LV_OPA_COVER, 0);
  lv_obj_set_style_text_font(lblIcon, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(lblIcon, lv_color_white(), 0);
  lv_label_set_text(lblIcon, LV_SYMBOL_WIFI);
  lv_obj_align(lblIcon, LV_ALIGN_CENTER, 0, -66);

  lv_obj_set_style_text_font(lblTitle, &lv_font_montserrat_14, 0);
  lv_label_set_text(lblTitle, "Config WiFi");
  lv_obj_align(lblTitle, LV_ALIGN_CENTER, 0, -30);

  lv_obj_set_style_text_font(lblValue, &lv_font_montserrat_20, 0);
  lv_label_set_text(lblValue, apSSID.c_str());
  lv_obj_align(lblValue, LV_ALIGN_CENTER, 0, 6);

  lv_obj_set_style_text_font(lblSub, &lv_font_montserrat_14, 0);
  String ip = WiFi.softAPIP().toString();
  lv_label_set_text(lblSub, ip.c_str());
  lv_obj_align(lblSub, LV_ALIGN_CENTER, 0, 40);
}

void updateCO2Label() {
  lv_label_set_text(lblValue, String(co2Value).c_str());
}

void setNormalUI() {
  // Clear any leftover splash transform so the number shows full size/opacity.
  lv_obj_set_style_opa(lblValue, LV_OPA_COVER, 0);
  lv_obj_set_style_transform_scale(lblValue, 256, 0);

  lv_obj_remove_flag(glow, LV_OBJ_FLAG_HIDDEN); // breathing rings back on
  lv_anim_delete(lblIcon, NULL); // stop any leftover icon animation
  lv_label_set_text(lblIcon, "");
  lv_label_set_text(lblTitle, "");
  lv_obj_set_style_text_font(lblValue, &lv_font_montserrat_48, 0);
  lv_obj_align(lblValue, LV_ALIGN_CENTER, 0, -4);
  lv_obj_set_style_text_font(lblSub, &lv_font_montserrat_14, 0);
  lv_label_set_text(lblSub, "ppm");
  lv_obj_align(lblSub, LV_ALIGN_CENTER, 0, 40);

  currentLevel = -1; // force theme apply on next refresh
  updateCO2Label();
}

// Refresh theme/speed when the CO2 level changes; always refresh the number.
void refreshNormal() {
  int lvl = co2Level(co2Value);
  if (lvl != currentLevel) {
    currentLevel = lvl;
    uint8_t s[3], e[3];
    getCO2Colors(co2Value, s, e);
    applyTheme(s, e, getRingDuration(co2Value));
  }
  updateCO2Label();
}

// ============================================================
// LED RING (custom WS2812FX mode: breath locked to the screen glow)
// ============================================================

// Custom WS2812FX effect: fill the whole ring with the segment colour scaled by
// breathLevel() -- the exact same oscillator the screen glow samples -- so the
// ring and the glow pulse in perfect lock-step. A small floor keeps the ring
// softly lit at the trough (gentle "leger" look) instead of fully off.
uint16_t breathSyncMode(void) {
  WS2812FX::Segment *seg = ws2812fx.getSegment();
  float level = 0.12f + 0.88f * breathLevel(); // 0.12 .. 1.0
  uint32_t c = seg->colors[0];
  uint8_t r = (uint8_t)(((c >> 16) & 0xFF) * level);
  uint8_t g = (uint8_t)(((c >> 8) & 0xFF) * level);
  uint8_t b = (uint8_t)((c & 0xFF) * level);
  uint32_t scaled = ws2812fx.Color(r, g, b);
  for (uint16_t i = seg->start; i <= seg->stop; i++)
    ws2812fx.setPixelColor(i, scaled);
  return 16; // re-render ~60 fps; brightness tracks the glow continuously
}

// Drive the WS2812FX engine (which runs breathSyncMode above).
void updateLeds() {
  if (ledRingReady)
    ws2812fx.service();
}

// ============================================================
// WIFI CONNECTION SCREENS (connecting / success / failure)
// ============================================================

// Keep the UI alive (LVGL + LED ring) for ms milliseconds, so the connect
// animations actually play during the otherwise-blocking connection flow.
static void uiHold(uint32_t ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    lv_timer_handler();
    updateLeds();
    delay(5);
  }
}

// "Connexion a <reseau>" screen: a big WiFi glyph gently pulsing (opacity
// breathes) while we try to associate, with the target SSID underneath.
void setConnectingUI(const String &ssid) {
  // Drive only the LED ring blue here; the screen stays clean (no glow mass).
  uint8_t s[3] = {0, 200, 255};
  uint8_t e[3] = {0, 0, 200};
  applyTheme(s, e, 2200);

  // Clean black background: hide the breathing rings on this screen.
  lv_obj_add_flag(glow, LV_OBJ_FLAG_HIDDEN);

  lv_anim_delete(lblIcon, NULL);
  lv_obj_set_style_text_font(lblIcon, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(lblIcon, lv_color_make(70, 170, 255), 0); // bleu
  lv_obj_set_style_opa(lblIcon, LV_OPA_COVER, 0);
  lv_label_set_text(lblIcon, LV_SYMBOL_WIFI);
  lv_obj_align(lblIcon, LV_ALIGN_CENTER, 0, -44);

  // Pulse the glyph's opacity in a loop to signal an in-progress attempt.
  lv_anim_t pulse;
  lv_anim_init(&pulse);
  lv_anim_set_var(&pulse, lblIcon);
  lv_anim_set_exec_cb(&pulse, labelOpaAnimCb);
  lv_anim_set_values(&pulse, 70, 255);
  lv_anim_set_duration(&pulse, 700);
  lv_anim_set_playback_duration(&pulse, 700);
  lv_anim_set_repeat_count(&pulse, LV_ANIM_REPEAT_INFINITE);
  lv_anim_start(&pulse);

  lv_obj_set_style_text_font(lblTitle, &lv_font_montserrat_14, 0);
  lv_label_set_text(lblTitle, "Connexion a");
  lv_obj_align(lblTitle, LV_ALIGN_CENTER, 0, 14);

  lv_obj_set_style_opa(lblValue, LV_OPA_COVER, 0);
  lv_obj_set_style_transform_scale(lblValue, 256, 0);
  lv_obj_set_style_text_font(lblValue, &lv_font_montserrat_20, 0);
  lv_label_set_text(lblValue, ssid.c_str());
  lv_obj_align(lblValue, LV_ALIGN_CENTER, 0, 44);

  lv_label_set_text(lblSub, "");
}

// Success / failure screen: green check or red cross, animated in with a
// little "pop", held on screen for a moment so the outcome is unmistakable.
void showConnectResult(bool ok, const String &ssid) {
  // Colour only the LED ring (green/red); the screen stays clean black.
  if (ok) {
    uint8_t s[3] = {60, 255, 120};
    uint8_t e[3] = {0, 180, 90};
    applyTheme(s, e, 3500);
  } else {
    uint8_t s[3] = {255, 70, 70};
    uint8_t e[3] = {200, 0, 0};
    applyTheme(s, e, 3500);
  }

  // Clean black background: no breathing rings behind the verdict.
  lv_obj_add_flag(glow, LV_OBJ_FLAG_HIDDEN);

  lv_anim_delete(lblIcon, NULL); // stop the connecting pulse
  lv_obj_set_style_opa(lblIcon, LV_OPA_COVER, 0);
  lv_obj_set_style_text_font(lblIcon, &lv_font_montserrat_48, 0);
  // Check vert, croix rouge.
  lv_obj_set_style_text_color(
      lblIcon, ok ? lv_color_make(60, 230, 130) : lv_color_make(255, 80, 80), 0);
  lv_label_set_text(lblIcon, ok ? LV_SYMBOL_OK : LV_SYMBOL_CLOSE);
  lv_obj_align(lblIcon, LV_ALIGN_CENTER, 0, -44);

  lv_obj_set_style_text_font(lblTitle, &lv_font_montserrat_14, 0);
  lv_label_set_text(lblTitle, ok ? "Connecte" : "Echec WiFi");
  lv_obj_align(lblTitle, LV_ALIGN_CENTER, 0, 14);

  lv_obj_set_style_opa(lblValue, LV_OPA_COVER, 0);
  lv_obj_set_style_text_font(lblValue, &lv_font_montserrat_20, 0);
  lv_label_set_text(lblValue, ssid.c_str());
  lv_obj_align(lblValue, LV_ALIGN_CENTER, 0, 44);

  lv_obj_set_style_text_font(lblSub, &lv_font_montserrat_14, 0);
  if (ok) {
    lv_label_set_text(lblSub, WiFi.localIP().toString().c_str());
    lv_obj_align(lblSub, LV_ALIGN_CENTER, 0, 70);
  } else {
    lv_label_set_text(lblSub, "");
  }

  // "Pop" the icon in (scale overshoot) so the verdict lands with a flourish.
  lv_obj_set_style_transform_pivot_x(lblIcon, lv_pct(50), 0);
  lv_obj_set_style_transform_pivot_y(lblIcon, lv_pct(50), 0);
  lv_obj_set_style_transform_scale(lblIcon, 80, 0);
  lv_anim_t pop;
  lv_anim_init(&pop);
  lv_anim_set_var(&pop, lblIcon);
  lv_anim_set_exec_cb(&pop, labelScaleAnimCb);
  lv_anim_set_values(&pop, 80, 256);
  lv_anim_set_duration(&pop, 600);
  lv_anim_set_path_cb(&pop, lv_anim_path_overshoot);
  lv_anim_start(&pop);

  uiHold(ok ? 1800 : 2200); // let the user read the outcome
  lv_anim_delete(lblIcon, NULL);
  lv_obj_set_style_transform_scale(lblIcon, 256, 0);
}

// ============================================================
// WIFI CREDENTIAL STORAGE (LittleFS)
// ============================================================

bool loadWifiCredentials() {
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS");
    return false;
  }
  if (!LittleFS.exists(WIFI_CONFIG_FILE)) {
    Serial.println("No WiFi config file found");
    return false;
  }
  File file = LittleFS.open(WIFI_CONFIG_FILE, "r");
  if (!file) {
    Serial.println("Failed to open WiFi config file");
    return false;
  }
  storedSSID = file.readStringUntil('\n');
  storedSSID.trim();
  storedPassword = file.readStringUntil('\n');
  storedPassword.trim();
  file.close();
  Serial.printf("Loaded WiFi: SSID=%s\n", storedSSID.c_str());
  return storedSSID.length() > 0;
}

bool saveWifiCredentials(const String &ssid, const String &password) {
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS");
    return false;
  }
  File file = LittleFS.open(WIFI_CONFIG_FILE, "w");
  if (!file) {
    Serial.println("Failed to create WiFi config file");
    return false;
  }
  file.println(ssid);
  file.println(password);
  file.close();
  Serial.printf("Saved WiFi: SSID=%s\n", ssid.c_str());
  return true;
}

bool deleteWifiConfig() {
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS");
    return false;
  }
  if (LittleFS.exists(WIFI_CONFIG_FILE)) {
    LittleFS.remove(WIFI_CONFIG_FILE);
    Serial.println("WiFi configuration deleted, restarting...");
    delay(1000);
    rp2040.reboot();
    return true;
  }
  Serial.println("No WiFi config file found");
  return false;
}

// ============================================================
// WIFI CONNECTION
// ============================================================

// Blocking STA connect used at boot. Keeps the splash animation + LED ring
// alive while waiting (the loop pumps LVGL), unlike a bare delay(). Returns
// true on association within timeoutMs. The caller decides what to do on
// failure (scan + retry, or AP fallback).
bool connectToWifi(const String &ssid, const String &password,
                   int timeoutMs = WIFI_CONNECT_TIMEOUT_MS) {
  Serial.printf("Connecting to WiFi: %s\n", ssid.c_str());

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < timeoutMs) {
    lv_timer_handler(); // keep the animation alive while connecting
    updateLeds();
    delay(5);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected! IP: %s\n",
                  WiFi.localIP().toString().c_str());
    return true;
  }
  Serial.println("WiFi connection failed");
  WiFi.disconnect();
  return false;
}

// Scan in STA mode and report whether the given SSID is currently visible.
// ModuleAir's fallback hinge: after a failed connect we only bother retrying
// if the network is actually around -- otherwise (AP off, device moved) we go
// straight to config mode instead of burning a second 15 s timeout.
bool wifiSsidIsVisible(const String &ssid) {
  Serial.printf("[WiFi] Scanning to check if '%s' is around...\n",
                ssid.c_str());
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks();
  bool found = false;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == ssid) {
      found = true;
      Serial.printf("[WiFi] SSID found in scan (RSSI %d dBm)\n", WiFi.RSSI(i));
      break;
    }
  }
  WiFi.scanDelete();
  if (!found)
    Serial.println("[WiFi] SSID not found in scan");
  return found;
}

// ============================================================
// CO2 READING
// ============================================================

// Non-blocking MH-Z19B reader. requestCO2() fires a measurement command;
// pollCO2() (called every loop) collects the 9-byte reply across iterations
// and updates co2Value + calls onCO2Updated() when a valid frame arrives.
// This keeps the animation from stalling on the UART (the old blocking read
// froze the screen ~200 ms every read).
static bool co2Pending = false;
static unsigned long co2RequestTime = 0;

void requestCO2() {
  while (mhzSerial.available())
    mhzSerial.read(); // flush stale bytes
  mhzSerial.write(CO2_CMD, 9);
  co2Pending = true;
  co2RequestTime = millis();
}

void pollCO2() {
  if (!co2Pending)
    return;

  if (mhzSerial.available() >= 9) {
    uint8_t r[9];
    mhzSerial.readBytes(r, 9); // returns immediately, bytes already buffered
    co2Pending = false;

#if CO2_DEBUG
    Serial.printf("[CO2] RX: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                  r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
#endif

    // Validate header + checksum (MH-Z19B: byte8 = 0xFF - sum(byte1..7) + 1)
    if (r[0] == 0xFF && r[1] == 0x86) {
      uint8_t cs = 0;
      for (int i = 1; i < 8; i++)
        cs += r[i];
      cs = (uint8_t)(0xFF - cs + 1);
      if (cs == r[8]) {
        uint16_t c = (r[2] << 8) | r[3];
        if (c > 300 && c < 5000) {
          co2Value = c;
          co2Sum += c; // feed the upload-window average (reset on each send)
          co2SampleCount++;
          onCO2Updated();
        }
#if CO2_DEBUG
        else
          Serial.printf("[CO2] value %u out of range (warming up?)\n", c);
#endif
      }
#if CO2_DEBUG
      else
        Serial.println("[CO2] checksum mismatch");
#endif
    }
#if CO2_DEBUG
    else
      Serial.println("[CO2] bad header -> check TX/RX wiring (crossed?) & 9600 baud");
#endif
  } else if (millis() - co2RequestTime > 300) {
#if CO2_DEBUG
    Serial.printf("[CO2] no reply (%d bytes) -> check wiring/power/5V\n",
                  mhzSerial.available());
#endif
    co2Pending = false; // no/short reply, drop and retry next cycle
  }
}

// ============================================================
// WEB SERVER (captive portal config + STA status page)
// ============================================================

static const char CONFIG_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="fr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>alba - Config WiFi</title>
<style>
*{box-sizing:border-box}body{margin:0;font-family:system-ui,sans-serif;
background:#1a1a2e;color:#e0e0e0;display:flex;justify-content:center;padding:18px}
.card{width:100%;max-width:420px}
h1{margin:.2em 0;font-size:1.5em;color:#4fc3f7}
.sub{margin:.2em 0 1em;color:#9aa}
.nets{margin-bottom:14px}
.net{display:flex;justify-content:space-between;align-items:center;padding:10px 12px;
margin:6px 0;background:#24243e;border-radius:8px;cursor:pointer;transition:.15s}
.net:hover{background:#33335a}
.rssi{color:#7a8;font-size:.85em}
input{width:100%;padding:11px;margin:6px 0;border:1px solid #44445e;border-radius:8px;
background:#16162a;color:#fff;font-size:1em}
.pw{position:relative}.pw button{position:absolute;right:6px;top:8px;width:auto;margin:0;
background:none;border:none;color:#9aa;font-size:1.2em;padding:4px}
button{width:100%;padding:12px;margin:8px 0 0;border:none;border-radius:8px;
background:#4fc3f7;color:#06121c;font-weight:600;font-size:1em;cursor:pointer}
.rescan{background:#33335a;color:#cde}
.muted{color:#778;font-size:.8em;text-align:center;margin-top:14px}
</style></head><body>
<div class="card">
<h1>alba</h1>
<p class="sub">Configuration du WiFi</p>
<div id="nets" class="nets">Recherche des reseaux...</div>
<form action="/save" method="POST">
<input id="ssid" name="ssid" placeholder="Nom du reseau (SSID)" required>
<div class="pw"><input id="pw" name="password" type="password" placeholder="Mot de passe">
<button type="button" onclick="tog()">&#128065;</button></div>
<button type="submit">Connecter</button>
</form>
<button class="rescan" onclick="scan()">&#8635; Rescanner</button>
<p class="muted">Reseau ouvert temporaire pour la configuration</p>
</div>
<script>
function esc(s){return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/"/g,'&quot;')}
function scan(){document.getElementById('nets').innerHTML='Recherche...';
fetch('/scan').then(r=>r.json()).then(render).catch(()=>{document.getElementById('nets').innerHTML='Erreur de scan'})}
function render(l){var h='';if(!l.length){h='Aucun reseau trouve';}
l.sort((a,b)=>b.rssi-a.rssi);
l.forEach(function(n){h+='<div class="net" onclick="pick(this)" data-s="'+esc(n.ssid)+'">'+
'<span>'+esc(n.ssid)+(n.encrypted?' &#128274;':'')+'</span>'+
'<span class="rssi">'+n.rssi+' dBm</span></div>'});
document.getElementById('nets').innerHTML=h}
function pick(el){document.getElementById('ssid').value=el.getAttribute('data-s');
document.getElementById('pw').focus()}
function tog(){var p=document.getElementById('pw');p.type=p.type=='password'?'text':'password'}
scan();
</script></body></html>)HTML";

// Scan networks (must be in STA mode) and return them as a JSON array.
String scanToJson() {
  int n = WiFi.scanNetworks();
  if (n < 0)
    n = 0;

  String json = "[";
  for (int i = 0; i < n && i < 20; i++) {
    if (i > 0)
      json += ",";
    json += "{\"ssid\":\"";
    String ssid = WiFi.SSID(i);
    ssid.replace("\\", "\\\\");
    ssid.replace("\"", "\\\"");
    json += ssid;
    json += "\",\"rssi\":";
    json += String(WiFi.RSSI(i));
    json += ",\"encrypted\":";
    json += (WiFi.encryptionType(i) != ENC_TYPE_NONE) ? "true" : "false";
    json += "}";
  }
  json += "]";

  WiFi.scanDelete();
  return json;
}

// The page fetches /scan: serve the list captured before the AP started
// (a live scan is unreliable while the cyw43 is in AP mode).
void handleScan() {
  server.send(200, "application/json", cachedScanJson);
}

void handleSave() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  ssid.trim();

  if (ssid.length() == 0) {
    server.send(400, "text/plain", "SSID manquant");
    return;
  }

  Serial.printf("[Web] /save SSID=%s (pwd len=%d)\n", ssid.c_str(),
                password.length());
  saveWifiCredentials(ssid, password);

  String page =
      "<!DOCTYPE html><html lang=\"fr\"><head><meta charset=\"utf-8\">"
      "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
      "<title>alba</title>"
      "<style>body{font-family:system-ui,sans-serif;background:#1a1a2e;"
      "color:#e0e0e0;text-align:center;padding:40px}h1{color:#4fc3f7}</style>"
      "</head><body><h1>alba</h1>"
      "<p>Connexion a <b>" + ssid + "</b>...</p>"
      "<p>L'appareil redemarre. Vous pouvez fermer cette page.</p>"
      "</body></html>";
  server.send(200, "text/html", page);

  delay(2000);
  rp2040.reboot();
}

void handleReset() {
  Serial.println("[Web] /reset");
  server.send(200, "text/html",
              "<!DOCTYPE html><html><body style='font-family:sans-serif;"
              "background:#1a1a2e;color:#eee;text-align:center;padding:40px'>"
              "<h2>WiFi oublie</h2><p>Redemarrage en mode configuration...</p>"
              "</body></html>");
  delay(1000);
  deleteWifiConfig();
}

void handleStatus() {
  String level;
  if (co2Value < 600)       level = "Excellent";
  else if (co2Value < 800)  level = "Bon";
  else if (co2Value < 1200) level = "Aerez";
  else                      level = "Ventilez !";

  String page =
      "<!DOCTYPE html><html lang=\"fr\"><head><meta charset=\"utf-8\">"
      "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
      "<meta http-equiv=\"refresh\" content=\"5\">"
      "<title>alba</title>"
      "<style>body{font-family:system-ui,sans-serif;background:#1a1a2e;color:#e0e0e0;"
      "text-align:center;padding:24px}h1{color:#4fc3f7;margin:.2em}"
      ".co2{font-size:3.2em;font-weight:700;margin:.1em}.lvl{color:#7a8;font-size:1.1em}"
      "table{margin:18px auto;border-collapse:collapse}td{padding:5px 12px;text-align:left}"
      "td:first-child{color:#9aa}form{margin-top:20px}"
      "button{padding:11px 18px;border:none;border-radius:8px;background:#b33;color:#fff;"
      "font-size:1em;cursor:pointer}</style></head><body>"
      "<h1>alba</h1>"
      "<div class=\"co2\">" + String(co2Value) + " <span style='font-size:.4em'>ppm</span></div>"
      "<div class=\"lvl\">" + level + "</div>"
      "<table>"
      "<tr><td>Reseau</td><td>" + storedSSID + "</td></tr>"
      "<tr><td>IP</td><td>" + WiFi.localIP().toString() + "</td></tr>"
      "<tr><td>Signal</td><td>" + String(WiFi.RSSI()) + " dBm</td></tr>"
      "<tr><td>Appareil</td><td>" + apSSID + "</td></tr>"
      "</table>"
      "<form action=\"/reset\" method=\"POST\">"
      "<button type=\"submit\">Oublier le WiFi</button></form>"
      "</body></html>";
  server.send(200, "text/html", page);
}

void handleRoot() {
  if (currentMode == MODE_CONFIG) {
    server.send_P(200, "text/html", CONFIG_HTML);
  } else {
    handleStatus();
  }
}

void handleCaptiveRedirect() {
  String url = "http://" + WiFi.softAPIP().toString() + "/";
  server.sendHeader("Location", url, true);
  server.send(302, "text/plain", "");
}

void handleNotFound() {
  if (currentMode == MODE_CONFIG) {
    handleCaptiveRedirect();
  } else {
    server.send(404, "text/plain", "Not found");
  }
}

void registerRoutes() {
  server.on("/", handleRoot);
  server.on("/scan", handleScan);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/reset", HTTP_POST, handleReset);
  server.on("/generate_204", handleCaptiveRedirect);
  server.on("/gen_204", handleCaptiveRedirect);
  server.on("/hotspot-detect.html", handleCaptiveRedirect);
  server.on("/connecttest.txt", handleCaptiveRedirect);
  server.on("/fwlink", handleCaptiveRedirect);
  server.onNotFound(handleNotFound);
}

// ============================================================
// MODE TRANSITIONS
// ============================================================

// Record a WiFi FSM transition (and reset the per-state timer the FSM reads).
void setWifiState(WifiState s) {
  wifiState = s;
  stateEnteredAt = millis();
}

// Start the HTTP server once. Routes are registered a single time; the server
// listens on whatever interface(s) are up (AP, STA, or both in AP+STA), so one
// begin() serves the captive portal and the LAN status page alike.
void startWebServer() {
  if (!routesRegistered) {
    registerRoutes();
    routesRegistered = true;
  }
  if (!serverStarted) {
    server.begin();
    serverStarted = true;
    Serial.println("[Web] HTTP server started");
  }
}

// Bring up the open configuration Access-Point + captive portal. A fresh STA
// scan is cached first for the portal's network list (a live scan is unreliable
// once the radio is in AP mode).
void apStart() {
  Serial.println("[WiFi] Scanning networks before AP...");
  WiFi.mode(WIFI_STA);
  cachedScanJson = scanToJson();
  Serial.printf("[WiFi] Scan done (%d bytes)\n", cachedScanJson.length());

  IPAddress apIP(AP_IP_OCT_1, AP_IP_OCT_2, AP_IP_OCT_3, AP_IP_OCT_4);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, subnet);
  bool apOk = WiFi.softAP(apSSID.c_str()); // open network (no password)

  Serial.printf("[AP] softAP(%s) -> %s\n", apSSID.c_str(),
                apOk ? "OK" : "FAILED");
  Serial.printf("[AP] IP: %s\n", WiFi.softAPIP().toString().c_str());

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  startWebServer();
}

// Switch the DISPLAY to the live ppm readout. Idempotent: a no-op if already
// showing it (used both when the config splash expires and when STA connects).
void showNormalDisplay() {
  if (currentMode == MODE_NORMAL)
    return;
  currentMode = MODE_NORMAL;
  setNormalUI();
  lastCO2Request = millis();
}

// We are associated to a known network: serve the LAN status page and show ppm.
void enterStaConnected() {
  Serial.printf("[WiFi] STA connected, IP %s\n",
                WiFi.localIP().toString().c_str());
  startWebServer();
  showNormalDisplay();
  // Schedule the first upload ~DATA_WARMUP_MS from now (not a full interval),
  // so a freshly (re)connected device reports quickly.
  lastDataSend = millis() - (DATA_SEND_INTERVAL - DATA_WARMUP_MS);
  setWifiState(WS_STA_CONNECTED);
}

// AP fallback: raise the config hotspot and show the "Config WiFi" screen.
void startConfigMode() {
  Serial.println("Entering configuration mode (WiFi AP)...");
  apStart();
  currentMode = MODE_CONFIG;
  setConfigUI();
  setWifiState(WS_AP_CONFIG);
}

// Every 10 min while in AP-data mode, try to rejoin the saved network WITHOUT
// dropping the hotspot, using concurrent AP+STA. Mirrors ModuleAir_V4.
void attemptBackgroundReconnect() {
  if (storedSSID.length() == 0) {
    setWifiState(WS_AP_DATA); // nothing saved; just re-arm the 10-min timer
    return;
  }
  Serial.printf("[WiFi] Background reconnect to '%s' (AP stays up)...\n",
                storedSSID.c_str());
  WiFi.mode(WIFI_AP_STA); // concurrent: keep the config hotspot, add a STA link
  WiFi.begin(storedSSID.c_str(), storedPassword.c_str());
  setWifiState(WS_AP_RETRYING);
}

// True whenever the config hotspot is up (so the loop keeps serving DNS + web).
bool isApActive() {
  return wifiState == WS_AP_CONFIG || wifiState == WS_AP_DATA ||
         wifiState == WS_AP_RETRYING;
}

// The WiFi connection state machine, ticked at 1 Hz from loop(). This is the
// ModuleAir_V4 mechanic, adapted to the arduino-pico WiFi API.
void wifiManagerLoop() {
  static unsigned long lastTick = 0;
  unsigned long now = millis();
  if (now - lastTick < 1000)
    return;
  lastTick = now;

  switch (wifiState) {
  case WS_STA_CONNECTED:
    // Detect a dropped link and start the 3-minute recovery window.
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] STA link lost -> reconnecting");
      WiFi.begin(storedSSID.c_str(), storedPassword.c_str());
      lastReconnectKickAt = now;
      setWifiState(WS_STA_RECONNECTING);
    }
    break;

  case WS_STA_RECONNECTING:
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] Reconnected (RSSI %d dBm)\n", WiFi.RSSI());
      setWifiState(WS_STA_CONNECTED);
    } else if (now - stateEnteredAt > STA_RECONNECT_WINDOW_MS) {
      // 3 min without recovery: fall back to the config hotspot.
      Serial.println("[WiFi] 3 min reconnect window expired -> AP fallback");
      WiFi.disconnect();
      startConfigMode();
    } else if (now - lastReconnectKickAt > STA_RECONNECT_KICK_MS) {
      // Re-kick the driver every 30 s in case its auto-retry backed off.
      Serial.println("[WiFi] still reconnecting (kick)");
      WiFi.begin(storedSSID.c_str(), storedPassword.c_str());
      lastReconnectKickAt = now;
    }
    break;

  case WS_AP_CONFIG:
    // After 3 min on the "Config WiFi" splash, switch the screen to ppm. The
    // hotspot keeps running so the user can still configure it.
    if (now - stateEnteredAt > AP_CONFIG_DURATION_MS) {
      Serial.println("[WiFi] 3 min in config -> show ppm (AP stays up)");
      showNormalDisplay();
      setWifiState(WS_AP_DATA);
    }
    break;

  case WS_AP_DATA:
    // Every 10 min, attempt to rejoin the saved network in the background.
    if (now - stateEnteredAt > AP_RETRY_INTERVAL_MS)
      attemptBackgroundReconnect();
    break;

  case WS_AP_RETRYING:
    if (WiFi.status() == WL_CONNECTED) {
      // Joined: tear the hotspot down cleanly and go STA-only.
      Serial.println("[WiFi] Background reconnect SUCCEEDED -> tearing down AP");
      delay(300); // let any in-flight HTTP response flush over the AP
      dnsServer.stop();
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
      enterStaConnected();
    } else if (now - stateEnteredAt > AP_RETRY_TIMEOUT_MS) {
      // 30 s and still no association: drop the STA half, keep the AP, and
      // re-arm the 10-minute timer for the next try.
      Serial.println(
          "[WiFi] Background reconnect timed out -> AP only, retry in 10 min");
      WiFi.mode(WIFI_AP);
      setWifiState(WS_AP_DATA);
    }
    break;
  }
}

// ============================================================
// DEVICE IDENTITY
// ============================================================

// Loud, unmistakable identity banner. TOKEN is the value to register on the
// AirCarto side (capteurs.capteurs.token). Printed at boot and on demand (type
// 'i' or 't' in the serial monitor) so it's always reachable even after the
// boot logs have scrolled away.
void printIdentity() {
  Serial.println();
  Serial.println("==================================================");
  Serial.println("  ALBA  -  identite du capteur (a enregistrer)");
  Serial.println("--------------------------------------------------");
  Serial.printf("  MAC        : %s%s\n", deviceMac.c_str(),
                tokenFromChipId ? "  (indispo -> chip id)" : "");
  Serial.printf("  TOKEN      : %s   <== a entrer cote serveur\n",
                deviceToken.c_str());
  Serial.printf("  device_id  : %s\n", deviceIdHex.c_str());
  Serial.printf("  SSID AP    : %s\n", apSSID.c_str());
  Serial.println("==================================================");
  Serial.println();
}

// ============================================================
// DATA UPLOAD (POST a measurement to the AirCarto Alba endpoint)
// ============================================================

// Encode an ASCII string as a hex string (each byte -> two uppercase hex
// digits). The endpoint recovers the token with PHP pack('H*', device_id), so
// the token must stay printable ASCII (it is -- it's the MAC as a hex string).
static String asciiToHex(const String &s) {
  String out;
  out.reserve(s.length() * 2);
  char b[3];
  for (size_t i = 0; i < s.length(); i++) {
    snprintf(b, sizeof(b), "%02X", (uint8_t)s[i]);
    out += b;
  }
  return out;
}

// Build the JSON body and POST it to DATA_SERVER_URL over HTTPS. Blocking (a
// few seconds incl. the TLS handshake), so the breathing animation pauses for
// the duration -- acceptable once a minute. Only call this while STA-connected.
void dataSenderSend() {
  int vMajor = 0, vMinor = 0, vPatch = 0;
  sscanf(FIRMWARE_VERSION, "%d.%d.%d", &vMajor, &vMinor, &vPatch);

  // Average of every valid reading taken since the last send (ModuleAir-Next-Gen
  // behaviour): integer mean = sum / count. -1 if the window had no valid sample
  // (the endpoint treats -1 as "not available"). Snapshot + reset the window
  // here so readings taken during the upload itself count toward the next one.
  int co2Avg = (co2SampleCount > 0) ? (int)(co2Sum / co2SampleCount) : -1;
  uint16_t samples = co2SampleCount;
  co2Sum = 0;
  co2SampleCount = 0;

  // Flat payload, exactly the fields api.aircarto.fr/capteurs/alba.php reads.
  // No GPS (no receiver) and no timestamp (no RTC) -> omitted; the server then
  // stamps the reading with its reception time, which the endpoint supports.
  String json = "{";
  json += "\"device_id\":\"" + deviceIdHex + "\"";
  json += ",\"signal_quality\":" + String(WiFi.RSSI());
  json += ",\"version\":" + String(PROTOCOL_VERSION);
  json += ",\"co2\":" + String(co2Avg);
  json += ",\"version_major\":" + String(vMajor);
  json += ",\"version_minor\":" + String(vMinor);
  json += ",\"version_patch\":" + String(vPatch);
  json += "}";

  Serial.printf("[Data] co2 moyenne=%d ppm sur %u echantillon(s) (live=%u)\n",
                co2Avg, samples, co2Value);

  // Raw HTTP/1.1 request built by hand over the TLS socket, so we can dump the
  // EXACT bytes sent and the EXACT bytes received (status line + every header +
  // body), verbatim, for debugging -- no library filtering, no interpretation.
  const char *host = "api.aircarto.fr";
  const char *path = "/capteurs/alba.php";
  const uint16_t port = 443;

  String req = String("POST ") + path + " HTTP/1.1\r\n";
  req += "Host: " + String(host) + "\r\n";
  req += "User-Agent: alba/" FIRMWARE_VERSION "\r\n";
  req += "Content-Type: application/json\r\n";
  req += "Content-Length: " + String(json.length()) + "\r\n";
  req += "Connection: close\r\n";
  req += "\r\n";
  req += json;

  Serial.println();
  Serial.println("================ REQUETE (brut) ================");
  Serial.print(req);
  Serial.println();
  Serial.println("================================================");

  WiFiClientSecure client;
  client.setInsecure(); // skip cert validation (mirrors ModuleAir_V4)
  client.setTimeout(12000);

  unsigned long t0 = millis();
  if (!client.connect(host, port)) {
    Serial.printf("[Data] connexion TLS %s:%u ECHOUEE (%.1fs)\n", host, port,
                  (millis() - t0) / 1000.0f);
    client.stop();
    return;
  }
  client.print(req);

  // Dump the raw response exactly as it arrives, until the server closes the
  // connection (Connection: close) or we hit the safety timeout.
  Serial.println("================ REPONSE (brut) ================");
  size_t nbytes = 0;
  unsigned long lastRx = millis();
  while (true) {
    while (client.available()) {
      Serial.write(client.read());
      nbytes++;
      lastRx = millis();
    }
    if (!client.connected() && !client.available())
      break; // server closed and buffer drained
    if (millis() - lastRx > 12000) {
      Serial.println("\n[Data] (timeout lecture reponse)");
      break;
    }
    delay(1);
  }
  client.stop();
  Serial.println();
  Serial.printf("=========== fin reponse (%u octets, %.1fs) ===========\n",
                (unsigned)nbytes, (millis() - t0) / 1000.0f);
}

// ============================================================
// STARTUP PHASE (sensor warmup while the boot animation plays)
// ============================================================

static unsigned long startupStart = 0;
static bool startupSensorSeen = false;

void finishStartup() {
  if (co2Value == 0) {
    Serial.println("WARNING: No valid CO2 data! Check sensor.");
    co2Value = 400;
  }
  Serial.println("Boot animation complete!");

#if SKIP_WIFI
  Serial.println("WiFi disabled (SKIP_WIFI) -> live CO2 display");
  showNormalDisplay();
  return;
#else
  // ModuleAir_V4 startup mechanic:
  //   stored SSID? -> try to join (attempt 1)
  //     fail -> is the SSID visible in a scan?
  //       yes -> retry once (attempt 2)
  //       no  -> config hotspot
  //   no stored SSID -> config hotspot
  if (loadWifiCredentials()) {
    Serial.printf("[WiFi] Stored SSID '%s', attempt 1/%d\n", storedSSID.c_str(),
                  WIFI_MAX_ATTEMPTS);
    setConnectingUI(storedSSID); // "Connexion a <reseau>" + pulsing WiFi glyph
    if (connectToWifi(storedSSID, storedPassword)) {
      showConnectResult(true, storedSSID); // green check, held a moment
      enterStaConnected();
      return;
    }
    if (wifiSsidIsVisible(storedSSID)) {
      Serial.printf("[WiFi] SSID visible, retry (attempt 2/%d)\n",
                    WIFI_MAX_ATTEMPTS);
      setConnectingUI(storedSSID);
      if (connectToWifi(storedSSID, storedPassword)) {
        showConnectResult(true, storedSSID);
        enterStaConnected();
        return;
      }
      Serial.println("[WiFi] retry failed -> config mode");
    } else {
      Serial.println("[WiFi] SSID not around -> config mode");
    }
    // Tried and failed: show the red verdict before dropping into config.
    showConnectResult(false, storedSSID);
  } else {
    Serial.println("[WiFi] No stored credentials -> config mode");
  }
  startConfigMode();
#endif
}

// Called by pollCO2() whenever a fresh, valid reading lands.
void onCO2Updated() {
  if (currentMode == MODE_STARTUP) {
    // Just note the first reading; the splash runs for a fixed, short time so
    // the intro animation always plays fully (the sensor keeps warming up and
    // its value is shown live once we switch to the normal display).
    if (!startupSensorSeen) {
      startupSensorSeen = true;
      Serial.printf("First valid reading: %d ppm\n", co2Value);
    }
  } else if (currentMode == MODE_NORMAL) {
    Serial.printf("CO2: %d ppm\n", co2Value);
    refreshNormal();
  }
}

// Leave the splash after a fixed, short duration (lets the intro play, then
// switches to the live display - no lingering on the boot screen).
void handleStartup() {
  if (millis() - startupStart > SPLASH_MS) {
    Serial.println("Splash done, switching to live display...");
    finishStartup();
  }
}

// ============================================================
// SERIAL RESET HELPER
// ============================================================

void checkSerialReset() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      Serial.println("\n*** RESET WiFi Configuration ***");
      deleteWifiConfig();
    } else if (cmd == 'i' || cmd == 'I' || cmd == 't' || cmd == 'T') {
      // Reprint the identity banner on demand (token to register server-side).
      printIdentity();
    }
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  // Give the USB-CDC link time to enumerate so the early boot logs aren't lost.
  // The PlatformIO monitor does not reset the Pico, so without this wait the
  // first prints often disappear. Bounded so the board still boots headless.
  unsigned long serialT0 = millis();
  while (!Serial && millis() - serialT0 < 3000)
    delay(10);
  delay(100);

  mhzSerial.begin(9600); // PIO-emulated UART on GP6/GP7 (pins set in ctor)

  ws2812fx.init();
  ws2812fx.setBrightness(LED_BRIGHTNESS);
  // Register our screen-synced breath as custom mode 0 and run it on the whole
  // ring. Colour is updated per CO2 level via ws2812fx.setColor() in applyTheme.
  ws2812fx.setCustomMode(breathSyncMode);
  ws2812fx.setSegment(0, 0, LED_COUNT - 1, FX_MODE_CUSTOM_0,
                      ws2812fx.Color(0, 200, 255), 1000, false);
  ws2812fx.start();
  ledRingReady = true;

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  if (!gfx->begin(62500000)) { // higher SPI clock -> faster flush, smoother
    Serial.println("gfx->begin() failed!");
    while (1)
      ;
  }
  gfx->fillScreen(0x0000);

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed, formatting...");
    LittleFS.format();
    LittleFS.begin();
  }

  // Derive the device token / AP SSID / device_id from the WiFi MAC, the same
  // way ModuleAir derives its id from the MAC. The token is the MAC as an
  // uppercase hex string (printable), which is what gets registered server-side
  // and shown in the SSID ("alba-<token>"); device_id is its ASCII-hex encoding.
#if !SKIP_WIFI
  WiFi.mode(WIFI_STA); // bring the cyw43 up so the MAC is readable
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  char macHex[13];
  snprintf(macHex, sizeof(macHex), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1],
           mac[2], mac[3], mac[4], mac[5]);
  deviceToken = String(macHex);
  char macColon[18];
  snprintf(macColon, sizeof(macColon), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],
           mac[1], mac[2], mac[3], mac[4], mac[5]);
  deviceMac = String(macColon);
#endif
  // Fallback to the flash chip id if the MAC is unavailable (radio off / no
  // radio): keep a stable, unique-per-board token either way.
  if (deviceToken.length() == 0 || deviceToken == "000000000000") {
    String fullId = String(rp2040.getChipID());
    deviceToken = fullId.length() >= 12 ? fullId.substring(fullId.length() - 12)
                                        : fullId;
    deviceToken.toUpperCase();
    tokenFromChipId = true;
  }
  apSSID = "alba-" + deviceToken;
  deviceIdHex = asciiToHex(deviceToken);
  printIdentity();

  // --- LVGL ---
  lv_init();
  lv_log_register_print_cb(lvLogCb);
  lv_tick_set_cb(lvMillisCb);
  lvDisplay = lv_display_create(SCREEN_W, SCREEN_H);
  lv_display_set_flush_cb(lvDisplay, lvFlushCb);
  lv_display_set_buffers(lvDisplay, lvDrawBuf, NULL, sizeof(lvDrawBuf),
                         LV_DISPLAY_RENDER_MODE_FULL);

  buildUI();
  setStartupUI();

  currentMode = MODE_STARTUP;
  startupStart = millis();
  requestCO2(); // kick off the first (non-blocking) sensor reading
  lastCO2Request = millis();
  Serial.println("Setup complete, entering loop");
}

// ============================================================
// LOOP (cooperative: LVGL + web + sensor + LEDs)
// ============================================================

void loop() {
  lv_timer_handler();
  updateLeds();
  pollCO2();

  // Fire a new (non-blocking) sensor request periodically.
  unsigned long reqInterval = (currentMode == MODE_STARTUP) ? 1500 : 3000;
  if (!co2Pending && millis() - lastCO2Request > reqInterval) {
    requestCO2();
    lastCO2Request = millis();
  }

  if (currentMode == MODE_STARTUP) {
    handleStartup();
  } else {
#if !SKIP_WIFI
    // Drive the WiFi connection state machine (connect/scan/retry/AP/reconnect)
    // and serve the captive portal / status page for whichever links are up.
    wifiManagerLoop();
    if (serverStarted) {
      if (isApActive())
        dnsServer.processNextRequest(); // captive portal only while the AP is up
      server.handleClient();
    }
    checkSerialReset();

    // Upload a measurement every DATA_SEND_INTERVAL while STA-connected. (In
    // AP-only mode there is no uplink, so nothing is sent.) The POST blocks for
    // a few seconds, briefly pausing the animation -- fine once a minute.
    if (WiFi.status() == WL_CONNECTED &&
        millis() - lastDataSend >= DATA_SEND_INTERVAL) {
      lastDataSend = millis();
      dataSenderSend();
    }
#endif
  }

  delay(5);
}

#include "ble_improv.h"

#include <BLE.h>
#include <BluetoothLock.h>
#include <WiFi.h>
#include <btstack.h>

// --- Improv WiFi UUIDs -------------------------------------------------------
// NEVER TOUCH: these are EXACTLY ModuleAir_V4's, hard-coded in the app
// (services/improv-ble.ts). One digit off and the sensor becomes invisible.
#define SERVICE_UUID "00467768-6228-2272-4663-277478268000"
#define CHAR_CURRENT_STATE "00467768-6228-2272-4663-277478268001"
#define CHAR_ERROR_STATE "00467768-6228-2272-4663-277478268002"
#define CHAR_RPC_COMMAND "00467768-6228-2272-4663-277478268003"
#define CHAR_RPC_RESULT "00467768-6228-2272-4663-277478268004"
#define CHAR_CAPABILITIES "00467768-6228-2272-4663-277478268005"
// Custom characteristic (outside the Improv spec): the networks seen by the
// sensor, one notification per network ("ssid\trssi\tsecure"), then "END".
#define CHAR_WIFI_SCAN "00467768-6228-2272-4663-277478268010"

// Improv states
#define STATE_AUTHORIZED 0x02
#define STATE_PROVISIONING 0x03
#define STATE_PROVISIONED 0x04

// Improv errors
#define ERROR_NONE 0x00
#define ERROR_INVALID_RPC 0x01
#define ERROR_UNKNOWN_RPC 0x02
#define ERROR_UNABLE_TO_CONNECT 0x03

// RPC commands
#define CMD_WIFI_SETTINGS 0x01
#define CMD_IDENTIFY 0x02

// --- Timings -----------------------------------------------------------------
// 120 ms between two list notifications: the value measured on ModuleAir.
// Below it an iPhone drops lines; above it 20 networks take over 3 s to arrive.
static const uint32_t NOTIFY_GAP_MS = 120;
// Join verdict: comfortably under the app's 30 s wait.
static const uint32_t CONNECT_TIMEOUT_MS = 20000;
// The first status() reads after a WiFi.begin() are noise: only trust a
// negative verdict after this grace period.
static const uint32_t CONNECT_EARLY_FAIL_MS = 2500;
// How long we stay VISIBLE after announcing a success. Not a mere safety net:
// the BLE link really drops during the WiFi switch (shared 2.4 GHz antenna),
// and the app then reconnects to READ the verdict (readVerdictWithReconnect).
// "Nobody is connected" does NOT mean "the app is done".
static const uint32_t LINGER_AFTER_SUCCESS_MS = 8000;

static const int MAX_NETS = 20;

// --- Characteristic with hand-driven notifications ---------------------------
// See port note 5 in ble_improv.h. put() stores the value (always readable by
// the app) and marks one notification as pending; flush() sends it as soon as
// BTstack has room. Both run on the main loop, under the Bluetooth lock, so the
// Bluetooth context never sees a half-written buffer.
class ImprovCharacteristic : public BLECharacteristic {
public:
  ImprovCharacteristic(const char *uuid, uint16_t props)
      : BLECharacteristic(BLEUUID(uuid), props) {}

  void put(const uint8_t *data, size_t len) {
    BluetoothLock lock;
    _charData = realloc(_charData, len ? len : 1);
    memcpy(_charData, data, len);
    _charLen = len;
    _pending = true;
  }

  void put(const char *s) { put((const uint8_t *)s, strlen(s)); }

  // True when the connected app has enabled notifications on this value.
  bool subscribed() const { return _notificationEnabled && con_handle; }

  bool pending() const { return _pending; }

  // Push the pending notification. Returns true once nothing is left to send.
  bool flush() {
    if (!_pending)
      return true;
    BluetoothLock lock;
    if (!_notificationEnabled || !con_handle) {
      _pending = false; // nobody listening: the value stays readable
      return true;
    }
    if (!att_server_can_send_packet_now(con_handle))
      return false; // try again on the next loop pass
    att_server_notify(con_handle, _valueHandle, (uint8_t *)_charData,
                      _charLen);
    _pending = false;
    return true;
  }

private:
  bool _pending = false;
};

// --- Module state ------------------------------------------------------------
static BLEService *improvService = nullptr;
static ImprovCharacteristic *charState = nullptr;
static ImprovCharacteristic *charError = nullptr;
static ImprovCharacteristic *charRpcCmd = nullptr;
static ImprovCharacteristic *charRpcResult = nullptr;
static ImprovCharacteristic *charCapabilities = nullptr;
static ImprovCharacteristic *charWifiScan = nullptr;

static BleImprovCredentialsCallback credentialsCb = nullptr;
static bool stackBuilt = false;
static bool bleRunning = false;
static String myDeviceId;

// Network list, copied from main.cpp then sent one notification at a time.
struct NetEntry {
  char ssid[33];
  int rssi;
  bool secure;
};
static NetEntry netList[MAX_NETS];
static int netCount = 0;
static int netCursor = -1; // >= 0: sending, index of the next line
static uint32_t nextNotifyAt = 0;
static bool sendingNets = false;
// List already served to THIS client; re-armed on disconnect so the next one
// gets a fresh list. We poll the subscription instead of relying on a
// subscribe callback (the proven ModuleAir mechanic).
static bool scanServed = false;

// --- What crosses the border between the Bluetooth context and loop() --------
// ABSOLUTE RULE: callbacks only raise scalar flags and copy bytes into fixed
// buffers. No String, no malloc, no Serial, no call into the BLE stack.
static volatile bool evtLinked = false;
static volatile bool evtUnlinked = false;
static volatile bool evtCreds = false;
static volatile bool evtBadRpc = false;
static volatile bool evtUnknownRpc = false;
static volatile bool evtIdentify = false;
static volatile bool clientLinked = false;

// Written by the callback, read by loop(). evtCreds is raised LAST: it
// publishes the buffers.
static char credSsid[33] = "";
static char credPass[65] = "";
static bool pendingConnect = false;

static bool connecting = false;
static uint32_t connectStart = 0;

static BleImprovUiState uiState = BLE_UI_OFF;
static char uiSsid[33] = "";
static uint32_t verdictAt = 0;

// --- Helpers -----------------------------------------------------------------

static void setState(uint8_t state) { charState->put(&state, 1); }

static void setError(uint8_t error) { charError->put(&error, 1); }

// RPC result: [total length, command, (len, bytes)*] -- the Improv layout the
// app decodes as is (parseRpcStrings).
static void sendRpcResult(uint8_t command, const String *strings, int count) {
  uint8_t data[64];
  size_t n = 2;
  for (int i = 0; i < count; i++) {
    size_t len = strings[i].length();
    if (n + 1 + len > sizeof(data))
      break;
    data[n++] = (uint8_t)len;
    memcpy(data + n, strings[i].c_str(), len);
    n += len;
  }
  data[0] = (uint8_t)(n - 1); // command byte + strings
  data[1] = command;
  charRpcResult->put(data, n);
}

// Back to "ready", without touching advertising or the GATT server.
static void resetSession() {
  sendingNets = false;
  scanServed = false;
  netCursor = -1;
  connecting = false;
  pendingConnect = false;
  credSsid[0] = 0;
  credPass[0] = 0;
  uiSsid[0] = 0;
}

// --- BTstack callbacks (Bluetooth context: flags only) -----------------------

class RpcCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    const uint8_t *val = (const uint8_t *)c->valueData();
    const size_t len = c->valueLen();
    if (!val || len < 2) {
      evtBadRpc = true;
      return;
    }

    const uint8_t command = val[1];
    if (command == CMD_WIFI_SETTINGS) {
      // Frame: [length, CMD, ssidLen, ssid..., pwLen, pw...]
      size_t pos = 2;
      if (pos >= len) {
        evtBadRpc = true;
        return;
      }
      const uint8_t ssidLen = val[pos++];
      if (pos + ssidLen > len) {
        evtBadRpc = true;
        return;
      }
      char ssid[sizeof(credSsid)] = "";
      size_t n = 0;
      for (uint8_t i = 0; i < ssidLen; i++) {
        const char ch = (char)val[pos++];
        if (n < sizeof(ssid) - 1)
          ssid[n++] = ch; // an SSID is 32 bytes at most
      }
      ssid[n] = 0;

      const uint8_t pwLen = (pos < len) ? val[pos++] : 0;
      char pass[sizeof(credPass)] = "";
      n = 0;
      for (uint8_t i = 0; i < pwLen && pos < len; i++) {
        const char ch = (char)val[pos++];
        if (n < sizeof(pass) - 1)
          pass[n++] = ch;
      }
      pass[n] = 0;

      memcpy(credSsid, ssid, sizeof(credSsid));
      memcpy(credPass, pass, sizeof(credPass));
      evtCreds = true; // LAST: publishes the buffers
    } else if (command == CMD_IDENTIFY) {
      evtIdentify = true;
    } else {
      evtUnknownRpc = true;
    }
  }
};
static RpcCallbacks rpcCallbacks;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    clientLinked = true;
    evtLinked = true;
  }
  void onDisconnect(BLEServer *) override {
    clientLinked = false;
    evtUnlinked = true;
    // No advertising call here: BTstack resumes advertising by itself once the
    // link is closed (hci.c re-enables it for the peripheral role).
  }
};
static ServerCallbacks serverCallbacks;

// --- Stack / advertising -----------------------------------------------------

// Scan response: the 128-bit service UUID. It does not fit in the 31-byte
// advertising packet next to the full name, and the NAME is what carries the
// device id (the sensor's identity server-side) -- it must never be truncated.
// BTstack keeps a pointer to this buffer, hence static.
static uint8_t scanResponse[18];

static void buildStack(const String &deviceName) {
  Serial.printf("[BLE Improv] building the stack as '%s' (free heap %u)\n",
                deviceName.c_str(), (unsigned)rp2040.getFreeHeap());
  BLE.begin(deviceName);

  improvService = new BLEService(BLEUUID(SERVICE_UUID));

  charState = new ImprovCharacteristic(CHAR_CURRENT_STATE, BLERead | BLENotify);
  charError = new ImprovCharacteristic(CHAR_ERROR_STATE, BLERead | BLENotify);
  charRpcCmd = new ImprovCharacteristic(CHAR_RPC_COMMAND, BLEWrite);
  charRpcResult = new ImprovCharacteristic(CHAR_RPC_RESULT, BLERead | BLENotify);
  charCapabilities = new ImprovCharacteristic(CHAR_CAPABILITIES, BLERead);
  charWifiScan = new ImprovCharacteristic(CHAR_WIFI_SCAN, BLERead | BLENotify);

  charRpcCmd->setCallbacks(&rpcCallbacks);

  uint8_t initState = STATE_AUTHORIZED;
  charState->put(&initState, 1);
  uint8_t initError = ERROR_NONE;
  charError->put(&initError, 1);
  uint8_t caps = 0x01; // bit 0 = "identify" command supported
  charCapabilities->put(&caps, 1);
  charWifiScan->put("pending");

  improvService->addCharacteristic(charState);
  improvService->addCharacteristic(charError);
  improvService->addCharacteristic(charRpcCmd);
  improvService->addCharacteristic(charRpcResult);
  improvService->addCharacteristic(charCapabilities);
  improvService->addCharacteristic(charWifiScan);

  BLE.server()->setCallbacks(&serverCallbacks);
  BLE.server()->addService(improvService);

  // Advertising data order is little-endian; BLEUUID stores it big-endian.
  BLEUUID uuid(SERVICE_UUID);
  scanResponse[0] = 17;
  scanResponse[1] =
      BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS;
  for (int i = 0; i < 16; i++)
    scanResponse[2 + i] = uuid.uuid128[15 - i];

  stackBuilt = true;
  Serial.printf("[BLE Improv] stack built (free heap %u)\n",
                (unsigned)rp2040.getFreeHeap());
}

static void startAdvertising() {
  BluetoothLock lock;
  // Flags + complete local name ("Alba-AABBCCDDEEFF", 22 bytes): fits.
  BLE.startAdvertising(false);
  gap_scan_response_set_data(sizeof(scanResponse), scanResponse);
}

// --- Public API --------------------------------------------------------------

void bleImprovOnCredentials(BleImprovCredentialsCallback cb) {
  credentialsCb = cb;
}

bool bleImprovRunning() { return bleRunning; }

BleImprovUiState bleImprovUiState() { return uiState; }

const char *bleImprovSsid() { return uiSsid; }

bool bleImprovClientConnected() { return bleRunning && clientLinked; }

bool bleImprovBusy() {
  return bleRunning && (pendingConnect || connecting || evtCreds);
}

bool bleImprovCanStop() {
  if (!bleRunning)
    return true;
  // Verdict not out yet: stopping now = a sensor that joins the network while
  // the app reports a failure.
  if (connecting || pendingConnect)
    return false;
  if (charState->pending() || charRpcResult->pending())
    return false;
  if (uiState == BLE_UI_SUCCESS)
    return millis() - verdictAt > LINGER_AFTER_SUCCESS_MS;
  return true;
}

void bleImprovPushNetworks(const BleImprovNet *nets, int count) {
  // Never overwrite the list while it is being sent: the cursor points into it.
  if (sendingNets)
    return;
  netCount = 0;
  for (int i = 0; i < count && netCount < MAX_NETS; i++) {
    if (!nets[i].ssid || !nets[i].ssid[0])
      continue; // hidden network
    strncpy(netList[netCount].ssid, nets[i].ssid, 32);
    netList[netCount].ssid[32] = 0;
    netList[netCount].rssi = nets[i].rssi;
    netList[netCount].secure = nets[i].secure;
    netCount++;
  }
  Serial.printf("[BLE Improv] %d networks in reserve\n", netCount);
}

void bleImprovStart(const String &deviceName, const String &deviceId) {
  if (bleRunning)
    return;
  myDeviceId = deviceId;

  // Built once, kept for life: tearing a BLE stack down and up again is the
  // classic source of crashes at the very end of a successful configuration
  // (it is what made the mini reboot on NimBLE).
  if (!stackBuilt)
    buildStack(deviceName);

  resetSession();
  evtLinked = evtUnlinked = evtCreds = false;
  evtBadRpc = evtUnknownRpc = evtIdentify = false;
  clientLinked = false;

  uint8_t s = STATE_AUTHORIZED;
  charState->put(&s, 1);
  uint8_t e = ERROR_NONE;
  charError->put(&e, 1);

  startAdvertising();
  uiState = BLE_UI_ADVERTISING;
  bleRunning = true;
  Serial.printf("[BLE Improv] advertising as '%s' (free heap %u)\n",
                deviceName.c_str(), (unsigned)rp2040.getFreeHeap());
}

void bleImprovStop() {
  if (!bleRunning)
    return;
  bleRunning = false;
  resetSession();
  uiState = BLE_UI_OFF;
  {
    BluetoothLock lock;
    BLE.stopAdvertising();
  }
  Serial.println("[BLE Improv] advertising stopped");
}

void bleImprovLoop() {
  if (!bleRunning)
    return;

  const uint32_t now = millis();

  // --- 0. Drain the events raised by the Bluetooth context ----------------
  if (evtBadRpc) {
    evtBadRpc = false;
    setError(ERROR_INVALID_RPC);
  }
  if (evtUnknownRpc) {
    evtUnknownRpc = false;
    setError(ERROR_UNKNOWN_RPC);
  }
  if (evtIdentify) {
    evtIdentify = false;
    // The user has the device in front of them, with a screen: acknowledging
    // is enough, there is no ambiguity to lift.
    Serial.println("[BLE Improv] identify requested");
    setError(ERROR_NONE);
  }

  if (evtLinked) {
    evtLinked = false;
    Serial.println("[BLE Improv] app connected");
    if (uiState == BLE_UI_ADVERTISING || uiState == BLE_UI_FAILED)
      uiState = BLE_UI_LINKED;
  }

  if (evtUnlinked) {
    evtUnlinked = false;
    Serial.println("[BLE Improv] app disconnected");
    if (!connecting && !pendingConnect) {
      resetSession();
      // A success SURVIVES the app leaving: leaving is precisely how it says
      // everything went fine.
      if (uiState != BLE_UI_SUCCESS)
        uiState = BLE_UI_ADVERTISING;
    } else {
      // A join attempt survives the link dropping (shared antenna during the
      // WiFi switch): the app reconnects to read the verdict.
      scanServed = false;
      sendingNets = false;
    }
  }

  if (evtCreds) {
    evtCreds = false;
    Serial.printf("[BLE Improv] credentials received - SSID: %s\n", credSsid);
    setError(ERROR_NONE);
    setState(STATE_PROVISIONING);
    pendingConnect = true;
  }

  // --- 1. One pending notification per characteristic, as room allows ----
  charState->flush();
  charError->flush();
  charRpcResult->flush();

  // --- 2. A client just subscribed to the network list? -------------------
  // THE SUBSCRIPTION IS THE REQUEST. We never scan here (see ble_improv.h).
  if (!sendingNets && !scanServed && charWifiScan->subscribed()) {
    scanServed = true;
    Serial.printf("[BLE Improv] network list requested (%d in reserve)\n",
                  netCount);
    netCursor = 0;
    nextNotifyAt = now;
    sendingNets = true; // netCount may be 0: "END" goes out alone
  }

  // --- 3. Send the list, one line per pass, only once the previous left ---
  if (sendingNets && charWifiScan->flush() &&
      (int32_t)(now - nextNotifyAt) >= 0) {
    if (netCursor < netCount) {
      const NetEntry &n = netList[netCursor];
      char line[48];
      int len = snprintf(line, sizeof(line), "%s\t%d\t%d", n.ssid, n.rssi,
                         n.secure ? 1 : 0);
      charWifiScan->put((const uint8_t *)line, len);
      netCursor++;
      nextNotifyAt = now + NOTIFY_GAP_MS;
    } else {
      // End marker: without it the app waits for more networks.
      charWifiScan->put("END");
      Serial.println("[BLE Improv] network list sent");
      sendingNets = false;
      netCursor = -1;
    }
    charWifiScan->flush();
  }

  // --- 4. Credentials received: hand them to main.cpp ---------------------
  if (pendingConnect) {
    pendingConnect = false;
    Serial.printf("[BLE Improv] joining '%s'...\n", credSsid);
    connecting = true;
    connectStart = now;
    uiState = BLE_UI_CONNECTING;
    strncpy(uiSsid, credSsid, sizeof(uiSsid) - 1);
    uiSsid[sizeof(uiSsid) - 1] = 0;
    // main.cpp saves them and calls WiFi.begin(). Strings are built HERE, on
    // the main loop -- never in a callback.
    if (credentialsCb)
      credentialsCb(String(credSsid), String(credPass));
    credPass[0] = 0; // do not keep the password around
    return;          // the verdict waits for the next pass
  }

  // --- 5. Success is an EVENT, not a state to settle in -------------------
  // Past the grace period we go back to plain advertising: otherwise a link
  // that blips right after association would leave the screen stuck on
  // "connected" (the loop the mini once had).
  if (uiState == BLE_UI_SUCCESS && (now - verdictAt) > LINGER_AFTER_SUCCESS_MS)
    uiState = BLE_UI_ADVERTISING;

  // --- 6. Join verdict ------------------------------------------------------
  if (!connecting)
    return;

  const uint32_t elapsed = now - connectStart;
  const int st = WiFi.status();

  if (st == WL_CONNECTED) {
    Serial.printf("[BLE Improv] connected! IP %s\n",
                  WiFi.localIP().toString().c_str());
    connecting = false;
    uiState = BLE_UI_SUCCESS;
    verdictAt = now;
    setState(STATE_PROVISIONED);
    // First string EMPTY on purpose: it is the ModuleAir's local URL slot, and
    // alba has no API for the app. Second: the device id = the token the
    // backend knows the sensor by, key of getPublicName.php.
    const String result[2] = {String(""), myDeviceId};
    sendRpcResult(CMD_WIFI_SETTINGS, result, 2);
    charState->flush();
    charRpcResult->flush();
    return;
  }

  const bool hardFail = (elapsed > CONNECT_EARLY_FAIL_MS) &&
                        (st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL);
  if (hardFail || elapsed > CONNECT_TIMEOUT_MS) {
    Serial.printf("[BLE Improv] join failed (status %d after %lu ms)\n", st,
                  (unsigned long)elapsed);
    connecting = false;
    uiState = BLE_UI_FAILED;
    setError(ERROR_UNABLE_TO_CONNECT);
    setState(STATE_AUTHORIZED); // ready for a new attempt
  }
}

/*
 * WiFi provisioning over Bluetooth LE ("Improv WiFi" protocol)
 *
 * What it gives the user: they open the AirCarto app, alba shows up in the
 * list of nearby sensors, they pick their network, type the password, done.
 * No hotspot to join, no browser to open. The captive portal stays available
 * as a fallback for anyone without the app.
 *
 * The dialect is EXACTLY the one of ModuleAir_V4 / ModuleAir mini
 * (src/ble_improv.cpp): same UUIDs, same bytes, same custom characteristic for
 * the network list. The app (services/improv-ble.ts) cannot tell them apart
 * while configuring; only the ADVERTISED NAME tells it which product it just
 * set up ("Alba-<token>" vs "ModuleAirMini-<token>" vs "ModuleAir-<token>").
 *
 * Port notes (ESP32/NimBLE -> Pico 2W/BTstack, arduino-pico BLE library):
 *
 *  1. NOTHING BLOCKS. Everything is a state machine woken by bleImprovLoop()
 *     on each loop() pass, so LVGL and the LED ring keep animating.
 *
 *  2. NO WIFI SCAN WHILE AN APP IS CONNECTED. The 2.4 GHz radio is shared by
 *     WiFi and Bluetooth on the CYW43. main.cpp scans once before raising the
 *     config hotspot and hands the list over (bleImprovPushNetworks); this
 *     module only serves it -- instantly, as a bonus.
 *
 *  3. NO LOCAL URL IN THE VERDICT. The ModuleAir answers ["http://<ip>",
 *     deviceId] because its web portal takes over. alba has no API the app can
 *     drive, so it answers ["", deviceId] like the mini: announcing a URL would
 *     make the app show a dead link.
 *
 *  4. BTstack CALLBACKS ONLY RAISE FLAGS. They run in the Bluetooth context;
 *     allocating there while loop() touches the same objects corrupts the heap
 *     (learnt the hard way on the mini). Fixed-size buffers + volatile flags.
 *
 *  5. NOTIFICATIONS ARE SENT BY HAND. The library's setValue() queues a
 *     "can send now" request through ONE registration slot per characteristic:
 *     two updates in a row before the first goes out re-insert the same node in
 *     BTstack's list and corrupt it. We store the value, then push it ourselves
 *     with att_server_notify() when the stack says it can send, one at a time.
 *
 * Advertising lives only while alba is offline (config hotspot up): main.cpp
 * calls bleImprovStop() once it is back on a network, and the sensor becomes
 * invisible. The stack itself is built once and kept for life.
 */
#pragma once

#include <Arduino.h>

// What the screen should show. This module draws nothing: it PUBLISHES its
// state and main.cpp repaints from loop().
enum BleImprovUiState {
  BLE_UI_OFF,         // not advertising (alba is online)
  BLE_UI_ADVERTISING, // advertising, nobody connected
  BLE_UI_LINKED,      // the app is connected: configuration in progress
  BLE_UI_CONNECTING,  // credentials received, WiFi trying to associate
  BLE_UI_SUCCESS,     // network joined
  BLE_UI_FAILED,      // join failed -- the app may retry without reconnecting
};

// One network to offer the app (copied from main.cpp's scan).
struct BleImprovNet {
  const char *ssid;
  int rssi;
  bool secure; // false = open network
};

// Credentials received. main.cpp stores them and starts the connection; this
// module then only READS WiFi.status() to report the verdict to the app.
typedef void (*BleImprovCredentialsCallback)(const String &ssid,
                                             const String &password);

// Start advertising as `deviceName` ("Alba-AABBCCDDEEFF"). `deviceId` is sent
// back once the network is joined: it is the sensor's token server-side, which
// lets the app identify it without the user typing anything. Idempotent.
void bleImprovStart(const String &deviceName, const String &deviceId);

// Stop advertising (the stack is kept). Idempotent.
void bleImprovStop();

// Call on every loop() pass. Never blocks. No-op when stopped.
void bleImprovLoop();

bool bleImprovRunning();

// True while an app is connected. Check before any WiFi scan.
bool bleImprovClientConnected();

// True from the credentials' arrival until the verdict. main.cpp uses it not to
// undo what we are doing (no background reconnect with the OLD credentials).
bool bleImprovBusy();

// True once BLE can be stopped without losing the verdict. The success state
// and the device id go out on the loop pass AFTER the association; and the
// link often drops during the WiFi switch, the app then RECONNECTS to read the
// verdict. So after a success we stay visible for a few seconds no matter what.
bool bleImprovCanStop();

// Hand over the list of networks to serve. Ignored while a list is being sent.
void bleImprovPushNetworks(const BleImprovNet *nets, int count);

void bleImprovOnCredentials(BleImprovCredentialsCallback cb);

BleImprovUiState bleImprovUiState();

// SSID of the attempt in progress (empty outside an attempt).
const char *bleImprovSsid();

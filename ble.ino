/**
 * BLE helpers — Cycling Power Meter
 * Target: Adafruit Feather nRF52832
 *
 * Services:
 *   BLEDfu  (0x) — OTA firmware update
 *   BLEDis        — Device Information
 *   BLEBas        — Battery
 *   BLEUart       — Nordic UART (two-way serial over BLE for calibration UI)
 *   CPS  0x1818   — Cycling Power Service (power + crank data)
 *
 * BLEUart replaces the previous custom log service. It provides the
 * GetUserInput() / printfLog() communication layer that calibrate.ino
 * and the command dispatcher in power.ino both depend on.
 *
 * connection_count is declared here (volatile, file-scope) and extern'd
 * in power.ino and calibrate.ino.
 */

#include <bluefruit.h>
#include <stdarg.h>

// ---------------------------------------------------------------------------
// BLE service / characteristic objects
// ---------------------------------------------------------------------------

// Standard helper services
BLEDfu  bledfu;   // OTA DFU — must be begun first per Adafruit recommendation
BLEDis  bledis;   // Device Information Service
BLEBas  blebas;   // Battery Service
BLEUart bleuart;  // Nordic UART Service (TX/RX over BLE)

// Cycling Power Service (CPS) — 0x1818
// UUID constants: Adafruit_nRF52_Arduino/libraries/Bluefruit52Lib/src/BLEUuid.h
BLEService        pwrService  = BLEService(UUID16_SVC_CYCLING_POWER);
BLECharacteristic pwrMeasChar = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLECharacteristic pwrFeatChar = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);
BLECharacteristic pwrLocChar  = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);
// CP Control Point (0x2A66): Write + Indicate only (per CPPS spec §3.2).
// Required by strict head units even when no procedures are meaningfully supported.
BLECharacteristic pwrCtrlChar = BLECharacteristic(0x2A66);

// ---------------------------------------------------------------------------
// Shared connection state
// Declared volatile here; extern'd in power.ino and calibrate.ino.
// printfLog() gates BLE UART output on connection_count > 0.
// connectCallback() re-starts advertising while under MAX_PRPH_CONNECTION.
// ---------------------------------------------------------------------------
#define MAX_PRPH_CONNECTION 2
volatile uint8_t connection_count = 0;

// ---------------------------------------------------------------------------
// bleSetup()
// ---------------------------------------------------------------------------
void bleSetup() {
  Bluefruit.autoConnLed(false);            // Suppress blue LED to save power
  Bluefruit.begin(MAX_PRPH_CONNECTION, 0);
  Bluefruit.setTxPower(-8);               // dBm: -40 -30 -20 -16 -12 -8 -4 0 4
  Bluefruit.setName(DEV_NAME);
  Bluefruit.setAppearance(0x0482);        // Cycling Power Sensor

  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  // OTA DFU must be begun before other services
  bledfu.begin();

  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  blebas.begin();

  // BLE UART — provides two-way serial channel for calibration UI and commands
  bleuart.begin();

  setupPwr();
  startAdv();

  Serial.println(F("BLE ready. Connect with Bluefruit app (UART mode) for commands."));
}

// ---------------------------------------------------------------------------
// startAdv()
// Primary advertising packet budget (31 bytes):
//   Flags:         3 bytes
//   CPS UUID:      4 bytes  (16-bit UUID AD record)
//   UART UUID:     4 bytes
//   Name:          9 bytes  (7 chars + 2 overhead) — moved to ScanResponse
//   Appearance:    4 bytes
//   Total primary: 11 bytes + appearance = 15 bytes, well within limit.
// ---------------------------------------------------------------------------
void startAdv(void) {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(pwrService);
  Bluefruit.Advertising.addService(bleuart);

  // Name in ScanResponse preserves primary packet space
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // units of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);              // advertise indefinitely
}

// ---------------------------------------------------------------------------
// setupPwr()
// Configures the four mandatory CPS characteristics.
// ---------------------------------------------------------------------------
void setupPwr(void) {
  pwrService.begin();

  // Power Measurement (0x2A63)
  // Payload (all little-endian):
  //   [0-1] Flags                        uint16  0x0020 = crank data present
  //   [2-3] Instantaneous Power          int16   watts
  //   [4-5] Cumulative Crank Revolutions uint16
  //   [6-7] Last Crank Event Time        uint16  1/1024-s units
  pwrMeasChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  pwrMeasChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pwrMeasChar.setFixedLen(PWR_MEAS_CHAR_LEN);
  pwrMeasChar.setCccdWriteCallback(cccdCallback);
  pwrMeasChar.begin();
  uint8_t initPwr[PWR_MEAS_CHAR_LEN] = { 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  pwrMeasChar.write(initPwr, sizeof(initPwr));

  // Cycling Power Feature (0x2A65): bit 3 = Crank Revolution Data Supported
  pwrFeatChar.setProperties(CHR_PROPS_READ);
  pwrFeatChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pwrFeatChar.setFixedLen(4);
  pwrFeatChar.begin();
  pwrFeatChar.write32(0x00000008);

  // Sensor Location (0x2A5D): 5 = Left Crank
  pwrLocChar.setProperties(CHR_PROPS_READ);
  pwrLocChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pwrLocChar.setFixedLen(1);
  pwrLocChar.begin();
  pwrLocChar.write8(5);

  // CP Control Point (0x2A66): Write + Indicate only (CPPS spec §3.2)
  pwrCtrlChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_INDICATE);
  pwrCtrlChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  pwrCtrlChar.setMaxLen(5);
  pwrCtrlChar.setWriteCallback(cpControlPointCallback);
  pwrCtrlChar.begin();
}

// ---------------------------------------------------------------------------
// blePublishPower()
// Sends a CPS Power Measurement notification.
//
// crankEventTime must be the timestamp frozen at the moment the revolution
// was detected (in 1/1024-s units), NOT millis() at the time of this call.
// Re-send the same value between revolutions — the head unit only
// recalculates cadence when crankRevs increments.
// ---------------------------------------------------------------------------
void blePublishPower(int16_t instantPwr, uint16_t crankRevs, uint16_t crankEventTime) {
  const uint16_t flags = 0x0020;  // bit 5: crank revolution data present

  uint8_t pwrdata[PWR_MEAS_CHAR_LEN];
  uint16ToLso(flags,          &pwrdata[0]);
  uint16ToLso((uint16_t)instantPwr, &pwrdata[2]);
  uint16ToLso(crankRevs,      &pwrdata[4]);
  uint16ToLso(crankEventTime, &pwrdata[6]);

  if (connection_count > 0) {
    if (!pwrMeasChar.notify(pwrdata, sizeof(pwrdata))) {
#ifdef DEBUG
      Serial.println(F("Notify failed — CCCD not enabled?"));
#endif
    }
  }
}

// ---------------------------------------------------------------------------
// blePublishBatt()
// ---------------------------------------------------------------------------
void blePublishBatt(uint8_t battPercent) {
  blebas.write(battPercent);
#ifdef DEBUG
  Serial.printf("Battery: %d%%\n", battPercent);
#endif
}

// ---------------------------------------------------------------------------
// GetUserInput()
// Reads a line from Serial or BLE UART into buf (caller provides char[64]).
// Non-blocking: returns immediately with buf[0]=='\0' if no input ready.
// Used by calibrate.ino and readUserInput() in power.ino.
// ---------------------------------------------------------------------------
void GetUserInput(char* buf) {
  buf[0] = '\0';

  // Serial input
  if (Serial.available()) {
    delay(10);  // Let the rest of the line arrive
    int i = 0;
    while (Serial.available() && i < 63) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') break;
      buf[i++] = c;
    }
    buf[i] = '\0';
    return;  // Prefer Serial over BLE UART if both arrive simultaneously
  }

  // BLE UART input
  int pos = 0;
  while (bleuart.available() && pos < 63) {
    buf[pos++] = (char)bleuart.read();
  }
  if (pos > 0) {
    // Strip trailing newline/carriage-return from BLE apps
    if (buf[pos-1] == '\n' || buf[pos-1] == '\r') pos--;
    buf[pos] = '\0';
  }
}

// ---------------------------------------------------------------------------
// printfLog()
// Formatted print to Serial and (when connected) BLE UART.
// Uses a static 256-byte buffer on the stack — safe for nRF52832.
// Chunked into 20-byte bleuart.write() calls to respect MTU limits.
// ---------------------------------------------------------------------------
void printfLog(const char* fmt, ...) {
  static char msg[256];

  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);

  if (n <= 0) return;
  if (n >= (int)sizeof(msg)) n = sizeof(msg) - 1;

  Serial.print(msg);

  if (connection_count > 0) {
    // BLE UART: send in ≤20-byte chunks with a small inter-chunk delay
    // to avoid overflowing the TX FIFO.
    const int CHUNK = 20;
    int sent = 0;
    while (sent < n) {
      int toSend = min(CHUNK, n - sent);
      bleuart.write((uint8_t*)(msg + sent), toSend);
      sent += toSend;
      if (sent < n) delay(10);
    }
  }
}

// ---------------------------------------------------------------------------
// CP Control Point callback (0x2A66)
// The head unit writes an op code; we must respond with an indication.
// Response format: [0x20][echo op code][result] {optional params}
// ---------------------------------------------------------------------------
void cpControlPointCallback(uint16_t connHandle, BLECharacteristic* chr,
                            uint8_t* data, uint16_t len) {
  if (len == 0) return;
  uint8_t opCode = data[0];

  uint8_t response[4] = { 0x20, opCode, 0x02, 0x00 };
  uint8_t responseLen = 3;

  switch (opCode) {
    case 0x01: response[2] = 0x01; break;  // Set Cumulative Value → Success
    case 0x02: response[2] = 0x01; break;  // Update Sensor Location → Success
    case 0x03:                              // Request Supported Locations → [Left Crank]
      response[2] = 0x01;
      response[3] = 0x05;
      responseLen = 4;
      break;
    default: response[2] = 0x02; break;    // Op Code Not Supported
  }

  pwrCtrlChar.indicate(connHandle, response, responseLen);

#ifdef DEBUG
  Serial.printf("CP ctrl: op=0x%02X result=0x%02X\n", opCode, response[2]);
#endif
}

// ---------------------------------------------------------------------------
// CCCD callback — called when the head unit enables/disables notifications
// ---------------------------------------------------------------------------
void cccdCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccdValue) {
#ifdef DEBUG
  Serial.printf("CCCD updated: 0x%04X\n", cccdValue);
  if (chr->uuid == pwrMeasChar.uuid) {
    Serial.println(chr->notifyEnabled()
                   ? F("Power notify enabled")
                   : F("Power notify disabled"));
  }
#endif
}

// ---------------------------------------------------------------------------
// connectCallback()
// ---------------------------------------------------------------------------
void connectCallback(uint16_t connHandle) {
  connection_count++;

  BLEConnection* conn = Bluefruit.Connection(connHandle);
  char peerName[32] = { 0 };
  conn->getPeerName(peerName, sizeof(peerName));

  digitalWrite(LED_PIN, HIGH);

  Serial.printf("Connected to: %s  (count=%d)\n",
                peerName[0] ? peerName : "(unknown)", connection_count);

  // Publish battery immediately so the app doesn't show stale value
  blePublishBatt(checkBatt());

  // Keep advertising so a second client (e.g. phone + head unit) can also connect
  if (connection_count < MAX_PRPH_CONNECTION) {
    Bluefruit.Advertising.start(0);
  }
}

// ---------------------------------------------------------------------------
// disconnectCallback()
// ---------------------------------------------------------------------------
void disconnectCallback(uint16_t connHandle, uint8_t reason) {
  (void)connHandle;
  if (connection_count > 0) connection_count--;

  digitalWrite(LED_PIN, LOW);

#ifdef DEBUG
  Serial.printf("Disconnected (reason=0x%02X, count=%d)\n", reason, connection_count);
#endif
}

// ---------------------------------------------------------------------------
// uint16ToLso()
// Pack a uint16_t into two bytes, least-significant octet first.
// Required for all BLE GATT characteristic fields.
// ---------------------------------------------------------------------------
void uint16ToLso(uint16_t val, uint8_t* out) {
  out[0] = val & 0xFF;
  out[1] = (val >> 8) & 0xFF;
}

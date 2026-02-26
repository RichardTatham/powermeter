/**
 * BLE helpers for transmitting cycling power meter data over Bluetooth Low Energy.
 * Targets the Adafruit Feather nRF52832.
 *
 * Advertising the Cycling Power Service (0x1818) with instantaneous power
 * and crank revolution data, compatible with Garmin 830 and other head units
 * implementing the Bluetooth SIG Cycling Power Profile.
 *
 * Crank Revolution Data (flags bit 5):
 *   - Cumulative Crank Revolutions: uint16, monotonically increasing, rolls
 *     over naturally at 65535.
 *   - Last Crank Event Time: uint16, in 1/1024-second units, rolls over at
 *     65535 (~64 seconds). This is the timestamp captured at the moment of the
 *     crank event — NOT the time of the BLE notification. The head unit derives
 *     cadence from the delta between consecutive event times:
 *       cadence (RPM) = 60 * 1024 / (lastEventTime_n - lastEventTime_n-1)
 *     The value must be frozen at the crank event moment and re-transmitted
 *     unchanged in every packet until the next crank event occurs.
 *
 * For the Adafruit BLE lib, see:
 *   https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/master/libraries/Bluefruit52Lib
 */

#include <bluefruit.h>
#include <stdarg.h>

/* Cycling Power Service Definitions
 * Cycling Power Service:               0x1818
 * Power Measurement Characteristic:    0x2A63
 * Cycling Power Feature Characteristic: 0x2A65
 * Sensor Location Characteristic:      0x2A5D
 *
 * UUID constants from:
 *   https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/src/BLEUuid.h
 */
BLEService        pwrService  = BLEService(UUID16_SVC_CYCLING_POWER);
BLECharacteristic pwrMeasChar = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLECharacteristic pwrFeatChar = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);
BLECharacteristic pwrLocChar  = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);

/*
 * A made up service to help development.
 */
BLEService        logService = BLEService(0xface);
BLECharacteristic logChar    = BLECharacteristic(0x1234);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

void bleSetup() {
  Bluefruit.begin(1, 0);
  Bluefruit.setName(DEV_NAME);

  // Set TX power immediately after begin(), before any service or advertising
  // setup. Accepted values (dBm): -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(-8);

  // Disable the blue connection LED to reduce power draw.
  Bluefruit.autoConnLed(false);

  // Register connect/disconnect callbacks.
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Start the BLE Battery Service
  blebas.begin();

  // Setup the Heart Rate Monitor service using
  // BLEService and BLECharacteristic classes
  setupPwr();

#ifdef BLE_LOGGING
  setupBleLogger();
#endif

  // Setup the advertising packet(s)
  startAdv();

#ifdef DEBUG
  Serial.println("BLE module configured and advertising.");
#endif
}

void startAdv(void) {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(pwrService);

#ifdef BLE_LOGGING
  Bluefruit.Advertising.addService(logService);
#endif

  Bluefruit.Advertising.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

/*
 * Configure the Cycling Power Service and its three mandatory characteristics.
 */
void setupPwr(void) {
  pwrService.begin();

  // Note: .begin() on a BLECharacteristic registers it under the most recently
  // begun BLEService. Always call pwrService.begin() before the characteristics.

  // --- Power Measurement (0x2A63) ---
  // Mandatory. NOTIFY for live updates; READ so the head unit can fetch the
  // last value on connection before the first notification arrives.
  //
  // Payload layout (all fields little-endian):
  //   [0-1]  Flags                        uint16  0x0020 = crank data present
  //   [2-3]  Instantaneous Power          int16   watts (signed)
  //   [4-5]  Cumulative Crank Revolutions uint16  rolls over at 65535
  //   [6-7]  Last Crank Event Time        uint16  1/1024 s units, rolls over at 65535
  pwrMeasChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  pwrMeasChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pwrMeasChar.setFixedLen(8);
  pwrMeasChar.begin();
  // Write a valid zeroed initial value so the attribute is non-empty on first
  // read. flags=0x0020 declares crank fields are present; all values zero.
  uint8_t initPwr[8] = { 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  pwrMeasChar.write(initPwr, sizeof(initPwr));

  // --- Cycling Power Feature (0x2A65) ---
  // Mandatory, Read-only. 32-bit little-endian bitmask of supported features.
  // Bit 3: Crank Revolution Data Supported — must match the crank fields in 2A63.
  pwrFeatChar.setProperties(CHR_PROPS_READ);
  pwrFeatChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pwrFeatChar.setFixedLen(4);
  pwrFeatChar.begin();
  pwrFeatChar.write32(0x00000008);  // bit 3 = Crank Revolution Data Supported

  // --- Sensor Location (0x2A5D) ---
  // Mandatory, Read-only. Single byte identifying sensor mounting position.
  // 5 = Left Crank.
  pwrLocChar.setProperties(CHR_PROPS_READ);
  pwrLocChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pwrLocChar.setFixedLen(1);
  pwrLocChar.begin();
  pwrLocChar.write8(5);  // 5 = Left Crank
}

/*
 * This service exists only to publish logs over BLE.
 */
#ifdef BLE_LOGGING
void setupBleLogger() {
  logService.begin();
  logChar.setProperties(CHR_PROPS_NOTIFY);
  logChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  logChar.setMaxLen(20);
  logChar.begin();
}
#endif

/*
 * Publish a Cycling Power Measurement notification.
 *
 * @param instantPwr     Instantaneous power in watts (signed)
 * @param crankRevs      Cumulative crank revolution count (rolls over at 65535)
 * @param crankEventTime Timestamp of the most recent crank event in 1/1024-second
 *                       units, already cast to uint16_t (rollover is expected and
 *                       handled correctly by the head unit).
 *                       This must be the frozen moment the crank event was detected
 *                       in power.ino — NOT millis() at the time of this call.
 *                       Re-send the same value between events; the head unit only
 *                       updates cadence when it sees the revolution count increment.
 */
void blePublishPower(int16_t instantPwr, uint16_t crankRevs, uint16_t crankEventTime) {
  /*
   * Cycling Power Measurement flags (uint16, little-endian):
   *   b0  Pedal Power Balance Present
   *   b1  Pedal Power Balance Reference
   *   b2  Accumulated Torque Present
   *   b3  Accumulated Torque Source
   *   b4  Wheel Revolution Data Present
   *   b5  Crank Revolution Data Present  <-- set
   *   b6  Extreme Force Magnitudes Present
   *   b7  Extreme Torque Magnitudes Present
   *   b8  Extreme Angles Present
   *   b9  Top Dead Spot Angle Present
   *   b10 Bottom Dead Spot Angle Present
   *   b11 Accumulated Energy Present
   *   b12 Offset Compensation Indicator
   */
  const uint16_t flags = 0x0020;  // bit 5: crank revolution data present

  uint8_t pwrdata[8];
  uint16ToLso(flags,          &pwrdata[0]);
  uint16ToLso(instantPwr,     &pwrdata[2]);
  uint16ToLso(crankRevs,      &pwrdata[4]);
  uint16ToLso(crankEventTime, &pwrdata[6]);

  if (pwrMeasChar.notify(pwrdata, sizeof(pwrdata))) {
#ifdef DEBUG
    Serial.print("Power: ");     Serial.print(instantPwr);
    Serial.print("W  Revs: ");   Serial.print(crankRevs);
    Serial.print("  Event: ");   Serial.println(crankEventTime);
#endif
  } else {
#ifdef DEBUG
    Serial.println("ERROR: Notify failed - CCCD not enabled or not connected!");
#endif
  }
}

/*
 * Update the Battery Service level (0-100%).
 */
void blePublishBatt(uint8_t battPercent) {
  blebas.write(battPercent);
#ifdef DEBUG
  Serial.printf("Updated battery percentage to %d%%\n", battPercent);
#endif
}

/*
 * Publish a short formatted log message over BLE (BLE_LOGGING builds only).
 * Capped at 19 printable characters + null terminator = 20 bytes total.
 */
#ifdef BLE_LOGGING
void blePublishLog(const char* fmt, ...) {
  static const int MAX = 20;
  static char msg[MAX];

  va_list args;
  va_start(args, fmt);
  int numBytes = vsnprintf(msg, MAX, fmt, args);  // safe: won't overflow buffer
  va_end(args);

  if (numBytes < 0) {
    Serial.println("BLE log: encoding error");
  } else if (numBytes >= MAX) {
    Serial.printf("BLE log: truncated to %d bytes\n", MAX - 1);
    logChar.notify(msg, MAX - 1);
  } else {
    if (!logChar.notify(msg, numBytes)) {
      Serial.println("BLE log: notify failed");
    }
#ifdef DEBUG
    else {
      Serial.printf("BLE log (%d bytes): %s\n", numBytes, msg);
    }
#endif
  }
}
#endif

/*
 * Callback: BLE central connected.
 */
void connectCallback(uint16_t connHandle) {
  BLEConnection* connection = Bluefruit.Connection(connHandle);
  char peerName[32] = { 0 };
  connection->getPeerName(peerName, sizeof(peerName));

  digitalWrite(LED_PIN, HIGH);

#ifdef DEBUG
  Serial.print("Connected to: ");
  Serial.println(peerName[0] != '\0' ? peerName : "(unknown)");
#endif
}

/*
 * Callback: BLE central disconnected.
 * @param reason  HCI status code from ble_hci.h
 */
void disconnectCallback(uint16_t connHandle, uint8_t reason) {
  (void) connHandle;
  (void) reason;

  digitalWrite(LED_PIN, LOW);

#ifdef DEBUG
  Serial.print("Disconnected, reason: 0x");
  Serial.println(reason, HEX);
  Serial.println("Resuming advertising...");
#endif
}

/*
 * Write a uint16_t into a 2-byte array in little-endian order (LSO first),
 * as required by all BLE GATT characteristic fields.
 */
void uint16ToLso(uint16_t val, uint8_t* out) {
  out[0] = val & 0xff;
  out[1] = (val >> 8) & 0xff;
}

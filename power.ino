/**
 * Cycling Power Meter — main file
 * Target: Adafruit Feather nRF52832
 *
 * File responsibilities:
 *   power.ino    — setup, main loop, power calculation, session data, commands
 *   ble.ino      — all BLE services, advertising, UART, printfLog, GetUserInput
 *   gyro.ino     — gyroSetup, getNormalAvgVelocity, getCircularVelocity, getCadence
 *   load_cell.ino — loadSetup, dataReadyISR, getAvgForce
 *   calibrate.ino — calibrateLoadCell, testBT
 *
 * Globals declared here and used by other .ino files:
 *   mpu, LoadCell        — hardware objects (gyro.ino, load_cell.ino)
 *   newLoadDataReady     — HX711 data-ready flag (load_cell.ino)
 *   newZrotDataReady     — gyro data-ready flag (gyro.ino)
 *   connection_count     — BLE connection count (ble.ino, declared there)
 *   nvram_settings, file — calibration storage (load_cell.ino, calibrate.ino)
 *   test_power, test_totalCrankRev, test_totalCrankRev_inc (calibrate.ino)
 *
 * RAM budget (nRF52832: 64 KB total):
 *   S132 SoftDevice:    ~8 KB
 *   sessionBuf[512]:     6 KB
 *   Stack + heap:       ~8 KB
 *   Remaining:         ~42 KB
 */

#include <Wire.h>
#include <bluefruit.h>
#include "MPU6050.h"
#include "HX711_ADC.h"
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

// ---------------------------------------------------------------------------
// Build flags
// ---------------------------------------------------------------------------
#define DEBUG                 // Serial debug output

// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
#define DEV_NAME               "JrvsPwr"
#define CRANK_RADIUS           0.175f   // metres
#define GYRO_OFFSET            -31      // Raw MPU6050 Z-axis bias
#define HOOKEDUPLOADBACKWARDS  1        // Set to -1 if force sign is naturally correct

#define VBATPIN      A7   // Feather built-in VBAT pin (/2 resistor divider)
#define LED_PIN      33
#define HX711_dout   A0
#define HX711_sck    A1

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
#define LOOP_DELAY            70        // ms — ~14 Hz effective sample rate
#define MIN_UPDATE_FREQ       1500      // ms — max BLE notification gap
#define STOPPED_BLE_UPDATE_MS 2000      // ms — keepalive rate when stopped
#define BATT_UPDATE_MS        300000UL  // 5 min
#define STAND_STILL_DPS       90.0f     // dps below which crank is considered stopped

// ---------------------------------------------------------------------------
// Calibration defaults (used when no saved file exists)
// ---------------------------------------------------------------------------
#define LOAD_OFFSET_DEFAULT    255904L
#define LOAD_MULT_DEFAULT      -2466.8989547f
#define CALIBRATIONS_FILENAME  "/calibrations.txt"
#define gn                     9.80665f  // m/s² — used in calibrate.ino
#define PWR_MEAS_CHAR_LEN      8         // BLE packet length — used in ble.ino

// ---------------------------------------------------------------------------
// Calibration struct — persisted to LittleFS by calibrate.ino
// ---------------------------------------------------------------------------
typedef struct {
  uint8_t calibrated;      // Magic: 0xAB = valid, else use defaults
  int16_t gyro_offset;
  long    load_offset;
  float   load_multiplier;
} nvram_settings_struct;

nvram_settings_struct nvram_settings;

// ---------------------------------------------------------------------------
// Session data — circular buffer
//
// 512 entries × 12 bytes = 6 KB. At 70 rpm covers ~7 min as a rolling
// window; oldest entries are silently overwritten. 'd' dumps chronologically.
// ---------------------------------------------------------------------------
#define SESSION_BUF_SIZE 512

typedef struct {
  uint16_t totalCrankRevs;
  uint16_t deltaTimeMs;  // ms per revolution, capped at 65535
  int16_t  power;        // watts
  uint16_t cadence;      // rpm, rounded
  uint16_t avgForce;     // Newtons, rounded
} sessionEntry_t;

static sessionEntry_t sessionBuf[SESSION_BUF_SIZE];
static uint16_t sessionHead  = 0;
static uint16_t sessionCount = 0;

// ---------------------------------------------------------------------------
// Hardware objects — declared here, used across gyro.ino and load_cell.ino
// ---------------------------------------------------------------------------
MPU6050 gyro;
HX711_ADC        LoadCell(HX711_dout, HX711_sck);
Adafruit_LittleFS_Namespace::File file(InternalFS);

// ---------------------------------------------------------------------------
// Shared volatile state
// connection_count declared in ble.ino; extern'd here
// ---------------------------------------------------------------------------
extern volatile uint8_t connection_count;

volatile uint8_t newLoadDataReady  = 0;  // Set by dataReadyISR in load_cell.ino
volatile uint8_t newZrotDataReady  = 0;  // Set by getZrot() in gyro.ino

// Test/fake mode — used by calibrate.ino → testBT()
int16_t  test_power             = 0;
uint16_t test_totalCrankRev     = 0;
uint16_t test_totalCrankRev_inc = 0;
bool     show_values            = false;

// Session statistics
static long  lastSessionStart      = 0;
static long  lastSessionEnd        = 0;
static long  lastSessionTotalCount = 0;
static float lastSessionTotalPower = 0.f;

// Persistent BLE cadence state
static uint16_t totalCrankRevs     = 0;
static uint16_t lastCrankEventTime = 0;  // 1/1024-s units, frozen at event

// ===========================================================================
// setup()
// ===========================================================================
void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(LED_PIN,      OUTPUT);
  digitalWrite(LED_PIN, LOW);

  lastSessionStart = millis();

  gyroSetup();    // gyro.ino
  loadSetup();    // load_cell.ino — inits HX711_ADC, loads calibration, attaches ISR
  bleSetup();     // ble.ino

  blePublishBatt(checkBatt());
  Serial.println(F("Ready. Send 'h' for help."));
}

// ===========================================================================
// loop()
// ===========================================================================
void loop() {
  static const float FMIN = -100000.f;
  static const float FMAX =  100000.f;

  static float   dps      = 0.f;
  static float   avgDps   = 0.f;
  static float   avgForce = 0.f;
  static float   maxForce = FMAX;
  static float   minForce = FMIN;
  static int16_t numPolls = 0;

  static long lastUpdate           = millis();
  static long lastInfrequentUpdate = millis();
  static long lastStopMessage      = millis();

  // Gyro sample — getZrot() returns abs(rad/s), fed through moving average
  dps     = getNormalAvgVelocity(dps);
  avgDps += dps;

  // Load cell sample
  newLoadDataReady = 0;
  float force = getAvgForce();
  if (force > maxForce) maxForce = force;
  if (force < minForce) minForce = force;
  avgForce += force;
  numPolls++;

  // Stopped: send zero-power keepalive
  bool pedaling = false;
  if (dps < STAND_STILL_DPS) {
    if ((millis() - lastStopMessage) >= STOPPED_BLE_UPDATE_MS) {
      if (test_power > 0 || test_totalCrankRev_inc > 0) {
        test_totalCrankRev += test_totalCrankRev_inc;
        printfLog("Fake: %dW rev=%d\n", test_power, test_totalCrankRev);
        blePublishPower(test_power, test_totalCrankRev, lastCrankEventTime);
      } else {
        blePublishPower(0, totalCrankRevs, lastCrankEventTime);
      }
      lastStopMessage = millis();
    }
  }

  // Moving: compute and publish once per detected revolution
  long timeNow = millis();

  if (numPolls > 2 && (timeNow - lastUpdate) > updateTime(dps, &pedaling)) {

    float computedDps   = avgDps / numPolls;
    float computedForce = (avgForce - maxForce - minForce) / (float)(numPolls - 2);
    float   mps         = getCircularVelocity(computedDps);
    int16_t power       = calcPower(mps, computedForce);

    if (pedaling) {
      uint16_t deltaMs = (uint16_t)min((long)65535L, timeNow - lastUpdate);
      totalCrankRevs++;
      lastCrankEventTime = (uint16_t)((timeNow * 1024UL) / 1000UL);

      uint16_t cadence = (deltaMs > 0)
                         ? (uint16_t)((60000UL + deltaMs / 2) / deltaMs)
                         : 0;
      sessionBuf[sessionHead] = {
        totalCrankRevs,
        deltaMs,
        power,
        cadence,
        (uint16_t)max(0.f, computedForce + 0.5f)
      };
      sessionHead = (sessionHead + 1) % SESSION_BUF_SIZE;
      if (sessionCount < SESSION_BUF_SIZE) sessionCount++;

      lastSessionTotalPower += power;
      lastSessionTotalCount++;
      lastSessionEnd = timeNow;
    }

    if (show_values) {
      printfLog("%.0fN %.2fm/s %dW %.0fdps %urev\n",
                computedForce, mps, power, computedDps, totalCrankRevs);
    }

#ifdef DEBUG
    Serial.print(F("Pwr: ")); Serial.print(power);
    Serial.print(F("W  dps: ")); Serial.println(computedDps);
#endif

    if (Bluefruit.connected()) {
      blePublishPower(power, totalCrankRevs, lastCrankEventTime);
    }

    // Reset accumulators; carry one sample forward to bridge the gap
    lastUpdate = timeNow;
    numPolls   = 1;
    avgDps     = dps;
    avgForce   = force;
    maxForce   = FMAX;
    minForce   = FMIN;

    if ((timeNow - lastInfrequentUpdate) > BATT_UPDATE_MS) {
      blePublishBatt(checkBatt());
      lastInfrequentUpdate = timeNow;
    }
  }

  readUserInput();

  delay(LOOP_DELAY);
}



// ===========================================================================
// calcPower()
// P = F × v × 2  (×2: one crank instrumented)
// ===========================================================================
int16_t calcPower(float mps, float forceN) {
  return (int16_t)(2.0f * forceN * mps);
}

// ===========================================================================
// updateTime()
// Target ms between BLE updates, tied to cadence.
// Sets *pedaling true when rotating faster than standstill threshold.
// ===========================================================================
float updateTime(float dps, bool* pedaling) {
  if (dps < 1.0f) {
    *pedaling = false;
    return (float)MIN_UPDATE_FREQ;
  }
  float del = min((float)MIN_UPDATE_FREQ, 1000.f * 360.f / dps);
  if (del < (float)MIN_UPDATE_FREQ) *pedaling = true;
  return del - 0.5f * (LOOP_DELAY + 30);
}

// ===========================================================================
// readUserInput()
// ===========================================================================
void readUserInput() {
  char buf[64] = { '\0' };
  GetUserInput(buf);  // ble.ino

  switch (buf[0]) {
    case 'h': printHelp();             break;
    case 'l': printLastSessionStats(); break;
    case 'd': printLastSessionData();  break;
    case 'c': calibrateLoadCell();     break;  // calibrate.ino
    case 'm':
      show_values = !show_values;
      printfLog(show_values ? "Monitor ON\n" : "Monitor OFF\n");
      break;
    case 'f':
      if (test_power > 0) {
        test_power = test_totalCrankRev = test_totalCrankRev_inc = 0;
        printfLog("Fake mode OFF\n");
      } else {
        testBT();  // calibrate.ino
      }
      break;
    default: break;
  }
}

// ===========================================================================
// printHelp()
// ===========================================================================
void printHelp() {
  printfLog("=== JrvsPwr ===\n");
  printfLog("Cal:    %s\n",  nvram_settings.calibrated == 0xAB ? "saved" : "defaults");
  printfLog("Offset: %ld\n", nvram_settings.load_offset);
  printfLog("Mult:   %.2f\n",nvram_settings.load_multiplier);
  printfLog("Batt:   %d%%\n",checkBatt());
  printfLog("h  Help\n");
  printfLog("l  Session stats\n");
  printfLog("d  Session CSV\n");
  printfLog("m  Monitor\n");
  printfLog("f  Fake BT\n");
  printfLog("c  Calibrate\n");
  printfLog("s  Sleep\n");
}

// ===========================================================================
// printLastSessionStats()
// ===========================================================================
void printLastSessionStats() {
  float duration_min = (lastSessionEnd - lastSessionStart) / 60000.f;
  if (duration_min > 0.f && lastSessionTotalCount > 0 && totalCrankRevs > 0) {
    printfLog("Duration: %.0f min\n", duration_min);
    printfLog("Avg pwr:  %.1f W\n",   lastSessionTotalPower / lastSessionTotalCount);
    printfLog("Cadence:  %.1f rpm\n", totalCrankRevs / duration_min);
  } else {
    printfLog("No stats yet.\n");
  }
}

// ===========================================================================
// printLastSessionData()
// Dumps session circular buffer as CSV over BLE UART, oldest entry first.
// ===========================================================================
void printLastSessionData() {
  if (sessionCount == 0) { printfLog("No data.\n"); return; }

  printfLog("revs,deltaMs,power,cadence,force\n");

  uint16_t start = (sessionCount < SESSION_BUF_SIZE) ? 0 : sessionHead;
  for (uint16_t i = 0; i < sessionCount; i++) {
    uint16_t idx = (start + i) % SESSION_BUF_SIZE;
    printfLog("%u,%u,%d,%u,%u\n",
              sessionBuf[idx].totalCrankRevs,
              sessionBuf[idx].deltaTimeMs,
              sessionBuf[idx].power,
              sessionBuf[idx].cadence,
              sessionBuf[idx].avgForce);
    delay(10);  // Prevent BLE TX FIFO overflow on large dumps
  }
  printfLog("---\n");
}

// ===========================================================================
// checkBatt()
// nRF52832: 12-bit ADC, 3.6 V ref, /2 divider on VBATPIN
// ===========================================================================
uint8_t checkBatt() {
  float v = (analogRead(VBATPIN) / 4096.0f) * 3.6f * 2.0f;
  if      (v > 4.1f) return 100;
  else if (v > 3.9f) return 90;
  else if (v > 3.7f) return 70;
  else if (v > 3.5f) return 40;
  else if (v > 3.3f) return 20;
  else               return 5;
}

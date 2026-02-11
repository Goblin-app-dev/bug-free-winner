/*
===============================================================================
 TENSTAR T-Display ESP32 — Board & Pinout Summary
===============================================================================

Board:
  - Vendor: TENSTAR
  - Form factor: T-Display (TTGO / LilyGO-style)
  - MCU: ESP32-D0WDQ6 (Classic ESP32, dual-core)
  - Crystal: 40 MHz
  - USB–Serial: CH9102
  - Flash: 16 MB (128 Mb), 3.3 V
  - PSRAM: None
  - Display: 1.14" ST7789 TFT (135 × 240)
  - Backlight: MOSFET-controlled via GPIO4 (digital ON/OFF only)
  - Known quirk: White square artifact during reset/boot
    → masked via full-screen white boot mask in setup()

Display (ST7789) → ESP32 GPIO (TFT_eSPI User_Setup.h):
  - SCLK  → GPIO18
  - MOSI  → GPIO19
  - CS    → GPIO5
  - DC    → GPIO16
  - RST   → GPIO23
  - BL    → GPIO4

Buttons:
  - TOP button    → GPIO35
      * Input-only pin
      * NO internal pull-up (external pull-up required)
      * Used for increment + EXT0 deep-sleep wake
  - BOTTOM button → GPIO0
      * Internal pull-up enabled
      * Used for decrement + long-hold sleep entry

Battery:
  - ADC pin → GPIO34
      * Input-only ADC
      * Voltage divider present on board (BAT_DIVIDER = 2.0)
      * ADC attenuation: 11 dB

Other:
  - Onboard LED → GPIO2

Firmware Notes:
  - No brightness menu, no PWM dimming (unreliable on this board).
  - Backlight is ON during operation, forced OFF + GPIO-hold in deep sleep.
  - GPIO4 is held LOW during deep sleep so the BL MOSFET cannot float ON.
  - Backlight polarity: HIGH = ON, LOW = OFF (ACTIVE HIGH)

Power Optimizations:
  - CPU: 40 MHz (down from 80 MHz) - workload is very light
  - Loop: 20ms delay (50 Hz polling) - buttons still feel instant
  - Battery sampling: 2 minutes when not charging (was 1 minute)
  - ADC samples: 25 per reading (was 50) - still provides good averaging
  - Smoothing: Alpha 0.3 (was 0.2) - compensates for longer sample interval

Battery Calibration Mode:
  - Calibration trigger at counter=88 is temporarily disabled
  - Log dump trigger at counter=12 is temporarily disabled (to avoid long blocking dump)
  - Logs battery voltage data to Serial and SPIFFS (/battery_cal.csv)
  - Data persists across reboots
  - Use Serial Monitor at 115200 baud to view real-time data
  - Retrieve log file after discharge cycle completes

Charging Detection:
  - Primary detection uses battery voltage threshold (>4.15V heuristic)
  - Full battery sampling runs at 800ms when charging, 2 minutes when not
  - A lightweight 2s fast-probe checks for plug/unplug transitions between full samples
  - On detected transition, firmware forces an immediate full sample and updates cadence
  - While charging, displayed battery % is frozen to avoid misleading voltage-based jumps

===============================================================================
*/

/*
  Tenstar ESP32 Modern Counter - Production Firmware
  --------------------------------------------------
  Boot sequence: white mask → BL off → clear to black → render first frame → BL on
  Battery UI redraw: Occurs ONLY on full battery sample ticks (fast-probe does not redraw)
  Non-charging sampling: Always 2 minutes regardless of battery level
  Charging animation: Phase advances on each sample tick (proportional to sample interval)
*/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <FreeSansBold60pt7b.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include "esp_adc_cal.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "SPIFFS.h"
#include "FS.h"

/* --------------------------------------------------------------
   GPIO Pin Assignments
   -------------------------------------------------------------- */

#define BTN_TOP        35  // Input-only, external pull-up required
#define BTN_BOTTOM     0   // Boot pin, internal pull-up enabled
#define BAT_ADC_PIN    34  // Input-only ADC pin
#define TFT_BL         4   // Backlight MOSFET control (ACTIVE HIGH)
#define LED_PIN        2   // Onboard status LED

/* --------------------------------------------------------------
   Display Configuration
   -------------------------------------------------------------- */

static const int SCREEN_W = 240;  // Display width after rotation=1
static const int SCREEN_H = 135;  // Display height after rotation=1

/* --------------------------------------------------------------
   Timing Constants (milliseconds)
   -------------------------------------------------------------- */

// Button timing
static const unsigned long BUTTON_SHORT_PRESS_MAX_MS = 800;   // Max duration for short press
static const unsigned long BUTTON_LONG_PRESS_MIN_MS  = 2000;  // Min duration for sleep trigger

// UI behavior timing
static const unsigned long LOCK_DELAY_MS = 5000;  // Idle time before max value locks

// Boot sequence timing
static const uint16_t WHITE_MASK_HOLD_MS = 300;  // Duration to show white boot mask
static const uint16_t BL_TRANSITION_DELAY_MS = 10;  // Delay during BL on/off transitions
static const uint16_t FRAME_SETTLE_DELAY_MS = 10;   // Delay for display buffer to settle
static const uint16_t SETUP_INIT_DELAY_MS = 100;    // Initial setup stabilization delay
static const uint16_t SLEEP_ENTRY_DELAY_MS = 50;    // Delay before entering deep sleep

// Animation timing
static const unsigned long BOOT_FRAME_MS = 33;  // ~30 FPS for boot animation

// Battery timing (OPTIMIZED for power savings)
static const uint32_t BAT_READ_MS_CHARGING = 800;     // Charging: ~1.25 Hz sampling
static const uint32_t BAT_READ_MS_NORMAL   = 120000;  // Non-charging: 2 minutes (was 60000)
static const uint32_t CHARGING_PROBE_MS    = 2000;    // Lightweight charging-state probe interval

// Main loop timing (OPTIMIZED for power savings)
static const unsigned long LOOP_DELAY_MS = 20;  // 50 Hz polling (was 10ms/100Hz)

/* --------------------------------------------------------------
   Battery Calibration Configuration
   -------------------------------------------------------------- */

// NOTE: Calibration trigger at counter=88 is temporarily disabled.
// Keep symbols/functions nearby for easy re-enable later.
// static const int CALIBRATION_MODE_VALUE = 88;   // Set counter to 88 to enable logging
// NOTE: Log dump trigger is temporarily disabled to avoid long blocking Serial dumps
// that can appear as a freeze when counter reaches 12. Keep constant/function code
// nearby for easy re-enable later.
// static const int LOG_DUMP_MODE_VALUE = 12;      // Set counter to 12 to dump log to Serial
static const char* CALIBRATION_FILE = "/battery_cal.csv";

/* --------------------------------------------------------------
   Battery Configuration
   -------------------------------------------------------------- */

#define BAT_DIVIDER    2.0    // Hardware voltage divider ratio
#define DEFAULT_VREF   1100   // ADC reference voltage (mV)

static const int BAT_ADC_SAMPLES = 25;  // Number of samples to average (OPTIMIZED: was 50)

/* --------------------------------------------------------------
   Animation Configuration
   -------------------------------------------------------------- */

// Phase advancement speeds (units per frame)
static const float CHARGING_SPEED = 0.12f;             // Phase delta per reference frame
static const float BOOT_SPEED = CHARGING_SPEED * 0.75f; // Boot animation 25% slower

// Reference timing for phase calculations
static const uint32_t CHARGING_REF_FRAME_MS = 100;  // Reference cadence (~10 FPS)

// Boot animation configuration
static const int BOOT_ANIMATION_CYCLES = 2;  // Number of complete bottom→top cycles

// Battery strip sprite configuration
static const int BATTERY_DOTS = 5;        // Number of battery level indicators
static const int BATTERY_DOT_SIZE = 6;    // Dot diameter (pixels)
static const int BATTERY_DOT_SPACING = 4; // Spacing between dots (pixels)
static const int BATTERY_STRIP_MARGIN_BOTTOM = 10;  // Distance from screen bottom

// Charging bolt icon dimensions
static const int CHARGING_BOLT_WIDTH = 8;
static const int CHARGING_BOLT_HEIGHT = 12;
static const int CHARGING_BOLT_OFFSET_Y = 20;  // Distance above battery dots

// Boot animation dot configuration
static const int BOOT_DOT_SIZE = 6;
static const int BOOT_DOT_SPACING = 10;

// Animation easing parameters
static const float BATTERY_PULSE_SCALE = 1.8f;  // Battery dot pulse magnitude
static const float BOOT_PULSE_SCALE = 2.2f;     // Boot dot pulse magnitude

/* --------------------------------------------------------------
   Display Layout Constants
   -------------------------------------------------------------- */

// Sprite dimensions
static const int ANIM_SPR_W = 40;     // Animation sprite width
static const int ANIM_SPR_H = SCREEN_H;  // Animation sprite height (full screen height)

// Battery sprite position (right edge)
static const int BAT_SPR_X = SCREEN_W - ANIM_SPR_W;
static const int BAT_SPR_Y = 0;

// Main number display area (partial redraw optimization)
static const int NUM_CLEAR_X = 0;
static const int NUM_CLEAR_Y = 0;
static const int NUM_CLEAR_W = 200;  // Width of number clearing area
static const int NUM_CLEAR_H = SCREEN_H;

// Text positioning
static const int NUM_DISPLAY_CENTER_X = 100;  // Horizontal center for number
static const int NUM_DISPLAY_CENTER_Y = 67;   // Vertical center for number

/* --------------------------------------------------------------
   Counter Configuration
   -------------------------------------------------------------- */

#define MIN_VAL        0             // Minimum counter value
static const int DEFAULT_START_VAL = 10;  // Initial counter value on boot

/* --------------------------------------------------------------
   Battery Voltage Mapping (calibrated for current battery)
   -------------------------------------------------------------- */

static const float VOLTAGE_MAX = 4.20f;  // Hard clamp max safe voltage

struct BatteryAnchor {
  float volts;
  int pct;
};

// Runtime-calibrated anchors for the currently-installed battery.
// Keep calibration capture/dump tooling disabled for production runtime,
// but retain those functions in source so this table can be regenerated
// and replaced when battery chemistry/aging changes.
static const BatteryAnchor SOC_TABLE[] = {
  {4.20f, 100},
  {4.00f,  75},
  {3.85f,  50},
  {3.70f,  25},
  {3.55f,  10},
  {3.40f,   0}
};
static const int SOC_TABLE_COUNT = sizeof(SOC_TABLE) / sizeof(SOC_TABLE[0]);

static const float CHARGING_DETECT_VOLTAGE = 4.15f;  // Heuristic charging detection

/* --------------------------------------------------------------
   Battery Percentage Thresholds
   -------------------------------------------------------------- */

static const int BATTERY_LEVEL_HEALTHY = 60;  // Above this: healthy color
static const int BATTERY_LEVEL_MID = 30;      // Above this: warning color
static const int BATTERY_LEVEL_LOW = 10;      // At or below: critical

/* --------------------------------------------------------------
   Color Definitions (RGB888)
   -------------------------------------------------------------- */

// Battery indicator colors
static const uint8_t COLOR_BATTERY_HEALTHY_R = 0x7F;
static const uint8_t COLOR_BATTERY_HEALTHY_G = 0xE7;
static const uint8_t COLOR_BATTERY_HEALTHY_B = 0xC4;

static const uint8_t COLOR_BATTERY_WARNING_R = 0xFF;
static const uint8_t COLOR_BATTERY_WARNING_G = 0xB7;
static const uint8_t COLOR_BATTERY_WARNING_B = 0x03;

static const uint8_t COLOR_BATTERY_CRITICAL_R = 0xEF;
static const uint8_t COLOR_BATTERY_CRITICAL_G = 0x23;
static const uint8_t COLOR_BATTERY_CRITICAL_B = 0x3C;

// Battery empty/unfilled dot color
static const uint8_t COLOR_BATTERY_EMPTY_GRAY = 80;

// Charging bolt color
static const uint8_t COLOR_CHARGING_BOLT_R = 255;
static const uint8_t COLOR_CHARGING_BOLT_G = 215;
static const uint8_t COLOR_CHARGING_BOLT_B = 0;

// Boot animation color (mint)
static const uint8_t COLOR_BOOT_ANIM_R = 0x7F;
static const uint8_t COLOR_BOOT_ANIM_G = 0xE7;
static const uint8_t COLOR_BOOT_ANIM_B = 0xC4;

/* --------------------------------------------------------------
   Gradient Color Points (for value-based coloring)
   -------------------------------------------------------------- */

struct RGB { uint8_t r, g, b; };

RGB COLOR_POINTS[] = {
  {52, 199, 89},    // Green (max value)
  {255, 214, 10},   // Yellow
  {255, 149, 0},    // Orange
  {255, 59, 48},    // Red-orange
  {255, 0, 0}       // Red (min value)
};
static const int NUM_COLOR_POINTS = 5;

/* --------------------------------------------------------------
   Battery Smoothing (OPTIMIZED for longer sample interval)
   -------------------------------------------------------------- */

static const float BATTERY_SMOOTH_ALPHA = 0.3f;  // Smoothing factor (was 0.2f)

/* --------------------------------------------------------------
   TFT instance + sprite buffer (NO PSRAM)
   -------------------------------------------------------------- */

TFT_eSPI tft;

TFT_eSprite animSprite = TFT_eSprite(&tft);
bool animSpriteReady = false;

/* --------------------------------------------------------------
   Counter and state variables
   -------------------------------------------------------------- */

int displayValue = DEFAULT_START_VAL;
int dynamicMaxValue = -1;
bool maxValueLocked = false;
unsigned long lastValueChange = 0;

// Bottom button press tracking
bool bottomPressed = false;
unsigned long bottomPressStart = 0;

// Top button press tracking
unsigned long topPressStart = 0;
bool topHeld = false;

/* --------------------------------------------------------------
   Battery state
   -------------------------------------------------------------- */

float batteryVoltage = VOLTAGE_MAX;  // Initialize to full voltage
int batteryPct = 100;
float batteryFillSmooth = 100.0f;

unsigned long lastBatteryReadMs = 0;
unsigned long lastChargingProbeMs = 0;
bool wasCharging = false;  // Track previous charging state for change detection

/* --------------------------------------------------------------
   Battery calibration state
   -------------------------------------------------------------- */

// bool calibrationModeActive = false;  // Temporarily disabled with calibration trigger
bool calibrationFileInitialized = false;
unsigned long calibrationSampleCount = 0;
// bool logDumpInProgress = false;  // Temporarily disabled with log dump feature

/* --------------------------------------------------------------
   Charging animation state (ADVANCES ONLY ON SAMPLE)
   -------------------------------------------------------------- */

float chargingPhase = 0.0f;

/* --------------------------------------------------------------
   First draw flag for optimization
   -------------------------------------------------------------- */

bool firstNumberDraw = true;

/* --------------------------------------------------------------
   ADC calibration
   -------------------------------------------------------------- */

esp_adc_cal_characteristics_t adc_chars;

/* --------------------------------------------------------------
   Forward declarations
   -------------------------------------------------------------- */

float easeOutCubic(float t);

void drawNumber();

float readBatteryVoltage();
float readBatteryVoltageFast();
void readBatteryVoltageDetailed(uint32_t &rawADC, uint32_t &adcMillivolts, float &batteryVolts);
int batteryPercentFromVoltage(float v);
uint16_t batteryColor(int pct);
bool isCharging();

// void initCalibrationFile();  // Temporarily disabled (see note near CALIBRATION_MODE_VALUE)
// void logBatteryCalibration(uint32_t rawADC, uint32_t adcMv, float batVolts, int batPct, bool charging);
// void dumpCalibrationLog();  // Temporarily disabled (see note near LOG_DUMP_MODE_VALUE)

static inline void ensureAnimSprite();
static void renderBatteryStrip();

void bootAnimationSprite();
static void showBootWhiteMaskHardCut(uint16_t holdMs);

void enterDeepSleep();

/* --------------------------------------------------------------
   Pre-setup backlight kill (prevents flash on power-up)

   Constructor attribute ensures this runs before setup().
   Sets backlight LOW (OFF) immediately to prevent power-on flash.
   -------------------------------------------------------------- */

__attribute__((constructor)) void preSetupBacklightKill() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);
}

/* --------------------------------------------------------------
   Backlight control (digital only, ACTIVE HIGH)

   Backlight polarity verified: HIGH = ON, LOW = OFF
   -------------------------------------------------------------- */

static inline void backlightOnFull() { digitalWrite(TFT_BL, HIGH); }
static inline void backlightOff()    { digitalWrite(TFT_BL, LOW); }

/* --------------------------------------------------------------
   Boot white mask + transition to black

   Purpose: Hide ROM bootloader artifact (white square) and
            eliminate visible tearing during screen clear.

   Sequence:
   1. BL ON → show white screen (masks ROM artifact)
   2. Hold white for specified duration
   3. BL OFF → user sees white due to LCD persistence
   4. Fill black → hidden transition (BL still off)
   5. Return with BL OFF → bootAnimationSprite() turns it on
                          after first frame is rendered
   -------------------------------------------------------------- */

static void showBootWhiteMaskHardCut(uint16_t holdMs) {
  backlightOnFull();
  tft.fillScreen(TFT_WHITE);
  delay(holdMs);

  // Backlight OFF before WHITE → BLACK transition (prevents visible tearing)
  backlightOff();
  delay(BL_TRANSITION_DELAY_MS);

  tft.fillScreen(TFT_BLACK);
  delay(BL_TRANSITION_DELAY_MS);
  // Leave BL OFF; boot animation turns it ON after first frame is rendered
}

/* --------------------------------------------------------------
   Gradient color interpolation

   Interpolates between COLOR_POINTS based on ratio [0.0, 1.0]
   Returns RGB565 color for TFT display
   -------------------------------------------------------------- */

uint16_t gradientColor(float ratio) {
  ratio = constrain(ratio, 0.0f, 1.0f);

  float segment = ratio * (NUM_COLOR_POINTS - 1);
  int idx = (int)floor(segment);
  float t = segment - idx;

  if (idx >= NUM_COLOR_POINTS - 1) {
    idx = NUM_COLOR_POINTS - 2;
    t = 1.0f;
  }

  RGB c1 = COLOR_POINTS[idx];
  RGB c2 = COLOR_POINTS[idx + 1];

  uint8_t r = c1.r + (uint8_t)((c2.r - c1.r) * t);
  uint8_t g = c1.g + (uint8_t)((c2.g - c1.g) * t);
  uint8_t b = c1.b + (uint8_t)((c2.b - c1.b) * t);

  return tft.color565(r, g, b);
}

/* --------------------------------------------------------------
   Get color for current counter value

   Returns white before max locks, then gradient based on
   proximity to max value (green → yellow → orange → red)
   -------------------------------------------------------------- */

uint16_t getColorForValue(int value) {
  if (!maxValueLocked) return TFT_WHITE;

  if (value == dynamicMaxValue) return gradientColor(0.0f);
  if (value <= 1) return gradientColor(1.0f);

  float ratio = (float)(dynamicMaxValue - value) / (float)(dynamicMaxValue - 1);
  return gradientColor(constrain(ratio, 0.0f, 1.0f));
}

/* --------------------------------------------------------------
   Main counter number rendering

   Uses partial clear optimization: only clears number area
   on subsequent draws (after first full-screen clear).
   -------------------------------------------------------------- */

void drawNumber() {
  if (firstNumberDraw) {
    tft.fillScreen(TFT_BLACK);
    firstNumberDraw = false;
  } else {
    tft.fillRect(NUM_CLEAR_X, NUM_CLEAR_Y, NUM_CLEAR_W, NUM_CLEAR_H, TFT_BLACK);
  }

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(getColorForValue(displayValue), TFT_BLACK);

  char buf[16];
  snprintf(buf, sizeof(buf), "%d", displayValue);

  tft.setFreeFont(&FreeSansBold60pt7b);
  tft.drawString(buf, NUM_DISPLAY_CENTER_X, NUM_DISPLAY_CENTER_Y);
  tft.setFreeFont(NULL);
}

/* --------------------------------------------------------------
   Battery voltage measurement (simple version)

   Averages BAT_ADC_SAMPLES readings for stability.
   Applies hardware voltage divider correction.
   Clamps to maximum safe voltage (4.20V).
   -------------------------------------------------------------- */

float readBatteryVoltage() {
  uint32_t sum = 0;

  for (int i = 0; i < BAT_ADC_SAMPLES; i++) {
    sum += analogRead(BAT_ADC_PIN);
  }

  uint32_t raw = sum / BAT_ADC_SAMPLES;
  uint32_t mv  = esp_adc_cal_raw_to_voltage(raw, &adc_chars);

  float volts = (mv * 0.001f) * BAT_DIVIDER;
  return min(volts, VOLTAGE_MAX);
}

/* --------------------------------------------------------------
   Battery voltage measurement (fast probe)

   Uses a single ADC read to cheaply probe charging-state changes
   between full sample intervals.
   -------------------------------------------------------------- */

float readBatteryVoltageFast() {
  uint32_t raw = analogRead(BAT_ADC_PIN);
  uint32_t mv  = esp_adc_cal_raw_to_voltage(raw, &adc_chars);

  float volts = (mv * 0.001f) * BAT_DIVIDER;
  return min(volts, VOLTAGE_MAX);
}

/* --------------------------------------------------------------
   Battery voltage measurement (detailed version for calibration)

   Returns all intermediate values for calibration logging:
   - rawADC: Average raw ADC reading
   - adcMillivolts: Calibrated millivolts from ADC
   - batteryVolts: Final battery voltage after divider correction
   -------------------------------------------------------------- */

void readBatteryVoltageDetailed(uint32_t &rawADC, uint32_t &adcMillivolts, float &batteryVolts) {
  uint32_t sum = 0;

  for (int i = 0; i < BAT_ADC_SAMPLES; i++) {
    sum += analogRead(BAT_ADC_PIN);
  }

  rawADC = sum / BAT_ADC_SAMPLES;
  adcMillivolts = esp_adc_cal_raw_to_voltage(rawADC, &adc_chars);
  batteryVolts = (adcMillivolts * 0.001f) * BAT_DIVIDER;

  // Clamp to max voltage
  if (batteryVolts > VOLTAGE_MAX) {
    batteryVolts = VOLTAGE_MAX;
  }
}

/* --------------------------------------------------------------
   Voltage to percentage conversion

   Uses voltage thresholds to estimate battery percentage.
   Not linear - based on typical Li-ion discharge curve.
   -------------------------------------------------------------- */

int batteryPercentFromVoltage(float v) {
  if (v >= SOC_TABLE[0].volts) return SOC_TABLE[0].pct;
  if (v <= SOC_TABLE[SOC_TABLE_COUNT - 1].volts) return SOC_TABLE[SOC_TABLE_COUNT - 1].pct;

  for (int i = 0; i < SOC_TABLE_COUNT - 1; i++) {
    const BatteryAnchor &hi = SOC_TABLE[i];
    const BatteryAnchor &lo = SOC_TABLE[i + 1];

    if (v <= hi.volts && v >= lo.volts) {
      float spanV = hi.volts - lo.volts;
      if (spanV <= 0.0f) return lo.pct;

      float t = (v - lo.volts) / spanV;
      float pctF = (float)lo.pct + t * (float)(hi.pct - lo.pct);
      return (int)(pctF + 0.5f);
    }
  }

  return 0;
}

/* --------------------------------------------------------------
   Battery status color mapping

   Returns color based on battery percentage:
   - ≥60%: Healthy (mint/cyan)
   - 30-59%: Warning (orange)
   - <30%: Critical (red)
   -------------------------------------------------------------- */

uint16_t batteryColor(int pct) {
  if (pct >= BATTERY_LEVEL_HEALTHY) {
    return tft.color565(COLOR_BATTERY_HEALTHY_R, COLOR_BATTERY_HEALTHY_G, COLOR_BATTERY_HEALTHY_B);
  }
  if (pct >= BATTERY_LEVEL_MID) {
    return tft.color565(COLOR_BATTERY_WARNING_R, COLOR_BATTERY_WARNING_G, COLOR_BATTERY_WARNING_B);
  }
  return tft.color565(COLOR_BATTERY_CRITICAL_R, COLOR_BATTERY_CRITICAL_G, COLOR_BATTERY_CRITICAL_B);
}

/* --------------------------------------------------------------
   Charging detection (heuristic)

   LIMITATION: This is voltage-based detection only.
   True charging detection would require current sensing.
   Assumes voltage >4.15V indicates active charging.
   -------------------------------------------------------------- */

bool isCharging() {
  return batteryVoltage > CHARGING_DETECT_VOLTAGE;
}

/* --------------------------------------------------------------
   Initialize battery calibration log file

   Creates CSV file with header if it doesn't exist.
   Called once when calibration mode is first activated.
   -------------------------------------------------------------- */

/*
 * TEMPORARILY DISABLED
 * --------------------
 * Calibration-mode trigger (counter=88) is currently disabled, so this helper
 * is commented out to prevent accidental reactivation through stale call sites.
 */
/*
void initCalibrationFile() {
  if (calibrationFileInitialized) return;

  // Check if file exists
  if (!SPIFFS.exists(CALIBRATION_FILE)) {
    fs::File f = SPIFFS.open(CALIBRATION_FILE, FILE_WRITE);
    if (f) {
      // Write CSV header
      f.println("timestamp_ms,sample_num,raw_adc,adc_mv,battery_volts,battery_pct,charging");
      f.close();

      Serial.println("=== BATTERY CALIBRATION LOG INITIALIZED ===");
      Serial.println("File: /battery_cal.csv");
      Serial.println("Format: timestamp_ms,sample_num,raw_adc,adc_mv,battery_volts,battery_pct,charging");
      Serial.println("===========================================");
    } else {
      Serial.println("ERROR: Failed to create calibration file");
    }
  } else {
    Serial.println("=== BATTERY CALIBRATION LOG RESUMED ===");
    Serial.println("Appending to existing /battery_cal.csv");
    Serial.println("========================================");
  }

  calibrationFileInitialized = true;
}
*/

/* --------------------------------------------------------------
   Log battery calibration data

   Appends one row to the CSV file and prints to Serial.
   Called on each battery sample when calibration mode is active.

   CSV Format:
   timestamp_ms, sample_num, raw_adc, adc_mv, battery_volts, battery_pct, charging
   -------------------------------------------------------------- */

/*
 * TEMPORARILY DISABLED
 * --------------------
 * Calibration-mode trigger (counter=88) is currently disabled.
 */
/*
void logBatteryCalibration(uint32_t rawADC, uint32_t adcMv, float batVolts, int batPct, bool charging) {
  unsigned long timestamp = millis();
  calibrationSampleCount++;

  // Format CSV line
  char logLine[128];
  snprintf(logLine, sizeof(logLine), "%lu,%lu,%u,%u,%.3f,%d,%d",
           timestamp,
           calibrationSampleCount,
           rawADC,
           adcMv,
           batVolts,
           batPct,
           charging ? 1 : 0);

  // Write to SPIFFS
  fs::File f = SPIFFS.open(CALIBRATION_FILE, FILE_APPEND);
  if (f) {
    f.println(logLine);
    f.close();
  } else {
    Serial.println("ERROR: Failed to write to calibration file");
  }

  // Print to Serial for real-time monitoring
  Serial.println(logLine);
}
*/

/* --------------------------------------------------------------
   Dump calibration log to Serial Monitor

   Reads the entire calibration log file and outputs it to Serial.
   This path is currently disabled to avoid long blocking serial dumps
   that can appear as a freeze on device.
   -------------------------------------------------------------- */

/*
 * TEMPORARILY DISABLED
 * --------------------
 * Full Serial dump can be very large and blocks execution long enough to look
 * like a freeze on device when counter reaches the trigger value.
 *
 * Keep this function in source for quick future re-enable.
 */
/*
void dumpCalibrationLog() {
  Serial.println("\n");
  Serial.println("============================================================");
  Serial.println("        BATTERY CALIBRATION LOG DUMP                        ");
  Serial.println("============================================================");
  Serial.println();

  if (!SPIFFS.exists(CALIBRATION_FILE)) {
    Serial.println("ERROR: Calibration file not found!");
    Serial.println("File path: /battery_cal.csv");
    Serial.println("No data has been logged yet.");
    Serial.println();
    return;
  }

  fs::File f = SPIFFS.open(CALIBRATION_FILE, FILE_READ);
  if (!f) {
    Serial.println("ERROR: Failed to open calibration file!");
    Serial.println();
    return;
  }

  size_t fileSize = f.size();
  Serial.printf("File size: %u bytes\n", fileSize);
  Serial.println();
  Serial.println("--- BEGIN CSV DATA ---");
  Serial.println();

  // Read and print file contents
  int lineCount = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.println(line);
      lineCount++;
    }
  }

  f.close();

  Serial.println();
  Serial.println("--- END CSV DATA ---");
  Serial.println();
  Serial.printf("Total lines: %d\n", lineCount);
  Serial.println();
  Serial.println("============================================================");
  Serial.println("  Copy the CSV data above and paste into a spreadsheet");
  Serial.println("  or save as battery_cal.csv for analysis");
  Serial.println("============================================================");
  Serial.println();
}
*/

/* --------------------------------------------------------------
   Cubic ease-out function

   Provides smooth deceleration for animation transitions.
   Input: t [0.0, 1.0]
   Output: eased value [0.0, 1.0]
   -------------------------------------------------------------- */

float easeOutCubic(float t) {
  if (t <= 0.0f) return 0.0f;
  if (t >= 1.0f) return 1.0f;
  float u = 1.0f - t;
  return 1.0f - (u * u * u);
}

/* --------------------------------------------------------------
   Sprite allocation helper

   Creates sprite on first call, reuses on subsequent calls.
   No PSRAM available on this board - uses heap memory.
   -------------------------------------------------------------- */

static inline void ensureAnimSprite() {
  if (animSpriteReady) return;

  animSprite.setColorDepth(16);
  animSprite.createSprite(ANIM_SPR_W, ANIM_SPR_H);
  animSprite.fillSprite(TFT_BLACK);
  animSpriteReady = true;
}

/* --------------------------------------------------------------
   Battery strip renderer

   Renders battery level indicator in right-side sprite:
   - 5 dots representing charge level
   - Charging bolt icon when charging
   - Pulsing animation on active dot when charging

   IMPORTANT: Does NOT read ADC - uses existing state variables.
   -------------------------------------------------------------- */

static void renderBatteryStrip() {
  ensureAnimSprite();

  animSprite.fillSprite(TFT_BLACK);

  int filled = map((int)batteryFillSmooth, 0, 100, 0, BATTERY_DOTS);

  const int x = ANIM_SPR_W / 2;

  int totalHeight = BATTERY_DOTS * (BATTERY_DOT_SIZE + BATTERY_DOT_SPACING) - BATTERY_DOT_SPACING;
  int bottomY = SCREEN_H - BATTERY_STRIP_MARGIN_BOTTOM;
  int topY = bottomY - totalHeight + (BATTERY_DOT_SIZE / 2);

  bool charging = isCharging();

  float active = chargingPhase;
  int mainDot = (int)active;
  float localT = active - mainDot;

  if (mainDot < 0) mainDot = 0;
  if (mainDot >= BATTERY_DOTS) mainDot = BATTERY_DOTS - 1;

  float ease = easeOutCubic(localT);

  // Draw charging bolt icon
  if (charging) {
    uint16_t boltColor = tft.color565(COLOR_CHARGING_BOLT_R, COLOR_CHARGING_BOLT_G, COLOR_CHARGING_BOLT_B);

    int boltX = x;
    int boltY = topY - CHARGING_BOLT_OFFSET_Y;

    // Upper triangle (top half of bolt)
    animSprite.fillTriangle(
      boltX,                          boltY,
      boltX - CHARGING_BOLT_WIDTH/2,  boltY + CHARGING_BOLT_HEIGHT/2,
      boltX + 1,                      boltY + CHARGING_BOLT_HEIGHT/2,
      boltColor
    );

    // Lower triangle (bottom half of bolt)
    animSprite.fillTriangle(
      boltX + 1,                      boltY + CHARGING_BOLT_HEIGHT/2,
      boltX + CHARGING_BOLT_WIDTH/2,  boltY + CHARGING_BOLT_HEIGHT/2,
      boltX,                          boltY + CHARGING_BOLT_HEIGHT,
      boltColor
    );
  }

  // Draw battery level dots
  for (int i = 0; i < BATTERY_DOTS; i++) {
    int dy = topY + (BATTERY_DOTS - 1 - i) * (BATTERY_DOT_SIZE + BATTERY_DOT_SPACING);
    int radius = BATTERY_DOT_SIZE / 2;

    uint16_t color;
    if (i < filled) {
      color = batteryColor(batteryPct);
    } else {
      color = tft.color565(COLOR_BATTERY_EMPTY_GRAY, COLOR_BATTERY_EMPTY_GRAY, COLOR_BATTERY_EMPTY_GRAY);
    }

    // Pulse effect on active charging dot
    if (charging && i == mainDot) {
      int pulseExtra = (int)(BATTERY_PULSE_SCALE * ease + 0.5f);
      radius += pulseExtra;
    }

    animSprite.fillCircle(x, dy, radius, color);
  }

  animSprite.pushSprite(BAT_SPR_X, BAT_SPR_Y);
}

/* --------------------------------------------------------------
   Boot animation (sprite-based; 2 complete bottom→top cycles)

   CRITICAL TIMING: Called with backlight OFF.
   Sequence:
   1. Render first animation frame
   2. Push to display (backlight still off)
   3. Turn backlight ON
   4. Continue animation

   This prevents visible black flash during boot.
   -------------------------------------------------------------- */

void bootAnimationSprite() {
  ensureAnimSprite();

  const int bootX = (SCREEN_W - ANIM_SPR_W) / 2;
  const int bootY = 0;

  const int x = ANIM_SPR_W / 2;

  int totalHeight = BATTERY_DOTS * (BOOT_DOT_SIZE + BOOT_DOT_SPACING) - BOOT_DOT_SPACING;
  int centerY = SCREEN_H / 2;

  int topY = centerY - totalHeight / 2 + (BOOT_DOT_SIZE / 2);
  int bottomY = topY + totalHeight - (BOOT_DOT_SIZE / 2);

  const uint16_t mint = tft.color565(COLOR_BOOT_ANIM_R, COLOR_BOOT_ANIM_G, COLOR_BOOT_ANIM_B);

  const float phaseEnd = (float)BATTERY_DOTS * (float)BOOT_ANIMATION_CYCLES;

  float phase = 0.0f;

  bool blTurnedOnAfterFirstFrame = false;

  while (phase < phaseEnd) {
    unsigned long frameStart = millis();

    animSprite.fillSprite(TFT_BLACK);

    int mainDot = (int)phase;
    int dotIndex = mainDot % BATTERY_DOTS; // 0 = bottom, (BATTERY_DOTS-1) = top
    float localT = phase - (float)mainDot;
    float ease = easeOutCubic(localT);

    // Draw all dots with pulse on active dot
    for (int i = 0; i < BATTERY_DOTS; i++) {
      int dy = bottomY - i * (BOOT_DOT_SIZE + BOOT_DOT_SPACING);
      int radius = BOOT_DOT_SIZE / 2;

      if (i == dotIndex) {
        int pulseExtra = (int)(BOOT_PULSE_SCALE * ease + 0.5f);
        radius += pulseExtra;
        animSprite.fillCircle(x, dy, radius, mint);
      } else {
        animSprite.fillCircle(x, dy, radius - 1, mint);
      }
    }

    // Push first frame to display while BL is still off
    animSprite.pushSprite(bootX, bootY);

    // After first frame is on screen, turn BL on
    if (!blTurnedOnAfterFirstFrame) {
      delay(FRAME_SETTLE_DELAY_MS); // Let frame settle in display buffer
      backlightOnFull();
      blTurnedOnAfterFirstFrame = true;
    }

    phase += BOOT_SPEED;

    unsigned long elapsed = millis() - frameStart;
    if (elapsed < BOOT_FRAME_MS) delay(BOOT_FRAME_MS - elapsed);
    yield();
  }

  // Clear animation sprite at end
  animSprite.fillSprite(TFT_BLACK);
  animSprite.pushSprite(bootX, bootY);
}

/* --------------------------------------------------------------
   Deep sleep entry

   Wake source: BTN_TOP (GPIO35) via EXT0 wakeup on LOW.
   Backlight is held LOW during sleep to prevent MOSFET float.
   -------------------------------------------------------------- */

void enterDeepSleep() {
  tft.fillScreen(TFT_BLACK);

  backlightOff();
  delay(SLEEP_ENTRY_DELAY_MS);

  // Configure GPIO hold to maintain backlight OFF state
  gpio_hold_dis((gpio_num_t)TFT_BL);
  gpio_deep_sleep_hold_dis();
  gpio_hold_en((gpio_num_t)TFT_BL);
  gpio_deep_sleep_hold_en();

  // Configure wake source
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_TOP, 0);  // Wake on LOW

  esp_deep_sleep_start();
}

/* --------------------------------------------------------------
   Setup
   -------------------------------------------------------------- */

void setup() {
  // Initialize Serial for calibration logging
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== TENSTAR T-Display Counter Starting ===");

  // Initialize SPIFFS for calibration data storage
  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR: SPIFFS mount failed");
  } else {
    Serial.println("SPIFFS mounted successfully");

    // Print SPIFFS info
    size_t totalBytes = SPIFFS.totalBytes();
    size_t usedBytes = SPIFFS.usedBytes();
    Serial.printf("SPIFFS: %u / %u bytes used\n", usedBytes, totalBytes);

    // Check if calibration file exists
    if (SPIFFS.exists(CALIBRATION_FILE)) {
      fs::File f = SPIFFS.open(CALIBRATION_FILE, FILE_READ);
      if (f) {
        Serial.printf("Existing calibration log found: %u bytes\n", f.size());
        f.close();
      }
    }
  }

  // Disable radios for power savings
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  btStop();
  esp_bt_controller_disable();

  // POWER OPTIMIZATION: Reduce CPU frequency to 40 MHz (was 80 MHz)
  // Workload is very light - 40 MHz is more than sufficient
  setCpuFrequencyMhz(40);
  Serial.println("CPU frequency: 40 MHz");

  // GPIO initialization
  pinMode(BTN_TOP, INPUT);
  pinMode(BTN_BOTTOM, INPUT_PULLUP);
  pinMode(BAT_ADC_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TFT_BL, OUTPUT);

  digitalWrite(LED_PIN, LOW);

  backlightOff();
  delay(SETUP_INIT_DELAY_MS);

  // Display initialization
  tft.init();
  tft.setRotation(1);  // 240×135 landscape orientation

  // Boot sequence: white mask → BL off → black → animation
  showBootWhiteMaskHardCut(WHITE_MASK_HOLD_MS);

  // ADC calibration for battery reading
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
  esp_adc_cal_characterize(
    ADC_UNIT_1,
    ADC_ATTEN_DB_11,
    ADC_WIDTH_BIT_12,
    DEFAULT_VREF,
    &adc_chars
  );

  // Boot animation (turns BL on after first frame)
  bootAnimationSprite();

  // Initialize state
  lastValueChange = millis();
  lastBatteryReadMs = 0;
  lastChargingProbeMs = 0;

  // Initial UI render
  firstNumberDraw = true;
  drawNumber();

  // Initial battery measurement and display
  batteryVoltage = readBatteryVoltage();
  batteryPct = batteryPercentFromVoltage(batteryVoltage);
  batteryFillSmooth = (float)batteryPct;
  wasCharging = isCharging();  // Initialize charging state tracker
  renderBatteryStrip();

  Serial.println("=== Setup Complete ===");
  Serial.println("Calibration trigger (counter=88) is temporarily disabled");
  Serial.println("Calibration log dump trigger is temporarily disabled");
  Serial.println();
}

/* --------------------------------------------------------------
   Main loop
   -------------------------------------------------------------- */

void loop() {
  unsigned long now = millis();

  // --- Top button: short press increments ---
  bool topPressedNow = (digitalRead(BTN_TOP) == LOW);

  if (topPressedNow && !topHeld) {
    topPressStart = now;
    topHeld = true;
  }

  if (!topPressedNow && topHeld) {
    if ((now - topPressStart) < BUTTON_SHORT_PRESS_MAX_MS) {
      displayValue++;
      lastValueChange = now;
      drawNumber();

      // NOTE: Calibration trigger intentionally disabled for now to prevent
      // accidental high-volume logging while investigating freeze issues.

      // NOTE: Log dump trigger intentionally disabled for now to prevent
      // long blocking serial output (can appear as freeze at value 12).

      // NOTE: Calibration mode exit path disabled together with trigger.
    }
    topHeld = false;
  }

  // --- Bottom button: short press decrements, long hold sleeps ---
  bool bottomNow = (digitalRead(BTN_BOTTOM) == LOW);

  if (bottomNow && !bottomPressed) {
    bottomPressed = true;
    bottomPressStart = now;
  }

  if (bottomNow && bottomPressed && (now - bottomPressStart > BUTTON_LONG_PRESS_MIN_MS)) {
    enterDeepSleep();
  }

  if (!bottomNow && bottomPressed) {
    if (now - bottomPressStart < BUTTON_LONG_PRESS_MIN_MS && displayValue > MIN_VAL) {
      displayValue--;
      lastValueChange = now;
      drawNumber();

      // NOTE: Calibration mode exit path disabled together with trigger.
    }
    bottomPressed = false;
  }

  // --- Auto-lock max value after idle period ---
  if (!maxValueLocked && now - lastValueChange >= LOCK_DELAY_MS) {
    dynamicMaxValue = displayValue;
    maxValueLocked = true;
    drawNumber();
  }

  // --- Battery sampling and charging state detection ---
  bool charging = isCharging();

  // Determine sample interval based on current charging state
  uint32_t readEvery = charging ? BAT_READ_MS_CHARGING : BAT_READ_MS_NORMAL;

  // Lightweight probe to detect charging state changes between full sample windows
  if (now - lastChargingProbeMs >= CHARGING_PROBE_MS) {
    lastChargingProbeMs = now;
    float probeVolts = readBatteryVoltageFast();
    bool probedCharging = (probeVolts > CHARGING_DETECT_VOLTAGE);

    if (probedCharging != wasCharging) {
      batteryVoltage = probeVolts;
      charging = probedCharging;
      readEvery = charging ? BAT_READ_MS_CHARGING : BAT_READ_MS_NORMAL;
    }
  }

  // Check if charging state changed - if so, force immediate sample
  bool chargingStateChanged = (charging != wasCharging);

  if (chargingStateChanged) {
    Serial.printf("Charging state changed: %s -> %s\n",
                  wasCharging ? "CHARGING" : "NOT CHARGING",
                  charging ? "CHARGING" : "NOT CHARGING");
    // Force immediate sample by resetting timer
    lastBatteryReadMs = now - readEvery;
  }

  // Battery sampling and UI redraw
  if (now - lastBatteryReadMs >= readEvery) {
    lastBatteryReadMs = now;

    // Read battery voltage with detailed breakdown for calibration
    uint32_t rawADC, adcMv;
    float newV;

    // Calibration logging path is temporarily disabled; keep normal read path only.
    newV = readBatteryVoltage();
    rawADC = 0;
    adcMv = 0;

    int newPct = batteryPercentFromVoltage(newV);

    batteryVoltage = newV;
    charging = isCharging();

    // Charging policy: freeze displayed battery percent while charging.
    // Voltage can rise early during charge and look "too full" too soon.
    if (!charging) {
      batteryPct = newPct;
    }

    // Exponential moving average for smooth visual transitions
    batteryFillSmooth = batteryFillSmooth * (1.0f - BATTERY_SMOOTH_ALPHA) + (float)batteryPct * BATTERY_SMOOTH_ALPHA;

    // Update charging state tracking after applying sample results
    wasCharging = charging;

    // NOTE: Calibration logging call disabled with calibration trigger.

    // Advance charging animation phase proportional to time elapsed
    if (charging) {
      // Calculate how many reference frames elapsed since last sample
      float frames = (float)readEvery / (float)CHARGING_REF_FRAME_MS;
      chargingPhase += (CHARGING_SPEED * frames);

      // Wrap phase to [0, BATTERY_DOTS)
      while (chargingPhase >= (float)BATTERY_DOTS) chargingPhase -= (float)BATTERY_DOTS;
      while (chargingPhase < 0.0f)  chargingPhase += (float)BATTERY_DOTS;
    } else {
      chargingPhase = 0.0f;
    }

    // Redraw battery UI (only happens here, at sample rate)
    renderBatteryStrip();
  }

  delay(LOOP_DELAY_MS);
}

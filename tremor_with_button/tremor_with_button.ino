/*
  ESP32 + GY-521 (MPU6050) Shake Detector + TXT Logger (SPIFFS)
  Version: BUTTON + SESSIONS, LED ON ONLY DURING SESSION, uses FREQUENCY to reject slow movements

  - Uses accel magnitude to detect "active" periods
  - Uses X-axis zero-crossings to estimate shake frequency
  - Only logs an event if:
      * duration >= MIN_SHAKE_DURATION_MS
      * avgIntensity >= MIN_AVG_INTENSITY_G
      * freqHz    >= MIN_SHAKE_FREQ_HZ

  Wiring:
    MPU6050 (GY-521) -> ESP32
      VCC  -> 3.3V
      GND  -> GND
      SDA  -> GPIO 21
      SCL  -> GPIO 22
      AD0  -> GND  (I2C addr 0x68)

    LED:
      GPIO 2 -> LED long leg (+)
      LED short leg (-) -> resistor (220–5.1k) -> GND

    BUTTON (momentary):
      One side -> GPIO 23
      Other side -> GND
      (We use INPUT_PULLUP, so button is active LOW)
*/

#include <Wire.h>
#include "FS.h"
#include "SPIFFS.h"
#include <math.h>

// ========================= I2C / MPU6050 SETTINGS =========================
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

const uint8_t MPU6050_ADDR         = 0x68;   // AD0 low
const uint8_t MPU6050_PWR_MGMT_1   = 0x6B;
const uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;

const float ACCEL_SENSITIVITY = 16384.0f;   // LSB/g for +/-2g

// ========================= SHAKE DETECTION SETTINGS =========================
float baselineGravityG = 1.0f;               // measured at startup

// Hysteresis thresholds – only stronger movement counts as "active"
const float SHAKE_START_THRESHOLD_G = 0.35f;  // start a shake when dynAcc >= this
const float SHAKE_END_THRESHOLD_G   = 0.20f;  // consider “weak” when dynAcc < this

// Timing: tune these to control how shakes group together
const unsigned long MIN_SHAKE_DURATION_MS     = 300;  // ignore very short motions
const unsigned long SAMPLE_INTERVAL_MS        = 10;   // 100 Hz sampling
const unsigned long QUIET_TIME_TO_END_MS      = 300;  // must be quiet this long to end a shake
const unsigned long MIN_GAP_BETWEEN_SHAKES_MS = 700;  // quiet gap before new shake can start

// Intensity filter: ignore gentle movements overall
const float MIN_AVG_INTENSITY_G = 0.25f;  // only log shakes with avgIntensity >= this

// Frequency filter: require tremor-like oscillation
const float AMP_ZERO_CROSS_THRESHOLD_G = 0.15f; // X-axis amplitude must exceed this to count a crossing
const float MIN_SHAKE_FREQ_HZ          = 3.0f;  // require at least ~3 Hz to be considered a "shake"

// ========================= SHAKE STATE =========================
bool          isShaking        = false;
unsigned long shakeStartMs     = 0;
float         shakeIntensitySum = 0.0f;
unsigned long shakeSampleCount  = 0;
unsigned long shakeEventIndex   = 0;
unsigned long lastShakeEndMs    = 0;
unsigned long lastAboveEndMs    = 0;   // last time dynAcc was above end threshold

// For frequency estimation using X-axis zero crossings
unsigned long zeroCrossCount = 0;
float         lastAxisVal    = 0.0f;
bool          lastAxisValid  = false;

// ========================= LED SETTINGS =========================
const int LED_PIN = 2;   // status LED (ON = session active)

// ========================= BUTTON + SESSION SETTINGS =========================
const int BUTTON_PIN = 23;   // momentary push button, to GND with INPUT_PULLUP

// Button debounce
int buttonState        = HIGH;  // last stable state (HIGH because of pull-up)
int lastButtonReading  = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_MS = 50;

// Session state
bool sessionActive = false;          // are we currently running a detection session?
int  sessionIndex  = 0;              // increments each session
int  shakesThisSession = 0;          // count shakes in current session
char sessionFilePath[32] = "/session_000.txt";   // updated per session

// ========================= FUNCTION DECLARATIONS =========================
bool initSPIFFS();
bool initMPU6050();
void calibrateBaseline();
void readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az);
void readAccelG(float &ax_g, float &ay_g, float &az_g);
void detectAndLogShake(float ax_g, float ay_g, float az_g, unsigned long nowMs);
void logShakeEvent(unsigned long startMs, unsigned long endMs,
                   float avgIntensity, float freqHz);
void appendLineToFile(const char *path, const String &line);

// Button/session helpers
void updateButton();
void toggleSession();
void startSession();
void endSession();

// ========================= SETUP =========================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("=== ESP32 Shake Detector + Logger (BUTTON + SESSIONS, FREQ FILTER) ===");

  // Filesystem
  if (!initSPIFFS()) {
    Serial.println("SPIFFS init failed. Logging may not work.");
  }

  // I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  // MPU6050
  if (!initMPU6050()) {
    Serial.println("MPU6050 init failed. Check wiring!");
  } else {
    Serial.println("MPU6050 init OK.");
  }

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // LED OFF initially (no session)

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lastButtonReading = digitalRead(BUTTON_PIN);
  buttonState       = lastButtonReading;

  Serial.println("Button configured on GPIO 23 (active LOW).");
  Serial.println("LED will be ON only when a detection session is active.");

  // Calibrate baseline
  calibrateBaseline();

  Serial.println("Setup complete. Press button to start a detection session.");
}

// ========================= MAIN LOOP =========================
void loop() {
  static unsigned long lastSampleMs = 0;
  unsigned long nowMs = millis();

  // Handle button start/stop with debounce
  updateButton();

  // Only sample + detect while a session is active
  if (sessionActive && (nowMs - lastSampleMs >= SAMPLE_INTERVAL_MS)) {
    lastSampleMs = nowMs;

    float ax_g, ay_g, az_g;
    readAccelG(ax_g, ay_g, az_g);

    detectAndLogShake(ax_g, ay_g, az_g, nowMs);
  }
}

// ========================= IMPLEMENTATION =========================

bool initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed.");
    return false;
  }
  Serial.println("SPIFFS mounted.");
  return true;
}

bool initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);  // wake up
  uint8_t error = Wire.endTransmission(true);

  if (error != 0) {
    Serial.print("MPU6050 I2C error: ");
    Serial.println(error);
    return false;
  }

  delay(100);
  return true;
}

void calibrateBaseline() {
  Serial.println("Keep device still. Calibrating baseline gravity...");
  const int numSamples = 200;
  float sumNorm = 0.0f;

  for (int i = 0; i < numSamples; i++) {
    float ax_g, ay_g, az_g;
    readAccelG(ax_g, ay_g, az_g);

    float norm = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    sumNorm += norm;

    delay(10);
  }

  baselineGravityG = sumNorm / numSamples;
  Serial.print("Baseline gravity magnitude (g): ");
  Serial.println(baselineGravityG, 4);
}

void readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU6050_ADDR, 6, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
}

void readAccelG(float &ax_g, float &ay_g, float &az_g) {
  int16_t ax, ay, az;
  readAccelRaw(ax, ay, az);

  ax_g = (float)ax / ACCEL_SENSITIVITY;
  ay_g = (float)ay / ACCEL_SENSITIVITY;
  az_g = (float)az / ACCEL_SENSITIVITY;
}

void detectAndLogShake(float ax_g, float ay_g, float az_g, unsigned long nowMs) {
  // Guard: do nothing if no active session
  if (!sessionActive) {
    return;
  }

  float norm = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  float dynamicAccel = fabs(norm - baselineGravityG);

  bool strongMotion = (dynamicAccel >= SHAKE_START_THRESHOLD_G);
  bool aboveEnd     = (dynamicAccel >= SHAKE_END_THRESHOLD_G);

  // ----- update zero-crossings on X-axis for frequency estimation -----
  // We only count crossings when amplitude is non-trivial.
  if (fabs(ax_g) >= AMP_ZERO_CROSS_THRESHOLD_G) {
    if (!lastAxisValid) {
      lastAxisVal   = ax_g;
      lastAxisValid = true;
    } else {
      int lastSign = (lastAxisVal >= 0.0f) ? 1 : -1;
      int currSign = (ax_g       >= 0.0f) ? 1 : -1;
      if (currSign != lastSign) {
        zeroCrossCount++;
      }
      lastAxisVal = ax_g;
    }
  }

  // ----- shake state machine -----
  if (!isShaking) {
    // Only start a new shake if:
    //  - motion is strong
    //  - we've been quiet long enough since last shake ended
    if (strongMotion && (nowMs - lastShakeEndMs >= MIN_GAP_BETWEEN_SHAKES_MS)) {
      isShaking         = true;
      shakeStartMs      = nowMs;
      shakeIntensitySum = dynamicAccel;
      shakeSampleCount  = 1;

      // reset frequency-related state for this new shake
      zeroCrossCount = 0;
      lastAxisValid  = false;
      lastAboveEndMs = nowMs;
      // Serial.println("Shake START");
    }
  } else {
    // We are currently in a shake
    if (aboveEnd) {
      // Still shaking or moderately active
      lastAboveEndMs    = nowMs;
      shakeIntensitySum += dynamicAccel;
      shakeSampleCount++;
    } else {
      // Below end threshold; check how long it has been "quiet"
      if (nowMs - lastAboveEndMs >= QUIET_TIME_TO_END_MS) {
        // End the shake only after sustained quiet time
        unsigned long duration = nowMs - shakeStartMs;
        if (duration >= MIN_SHAKE_DURATION_MS && shakeSampleCount > 0) {
          float avgIntensity = shakeIntensitySum / shakeSampleCount;

          // Estimate frequency from zero crossings on X-axis
          float durationSec = duration / 1000.0f;
          float cycles      = zeroCrossCount / 2.0f;  // roughly 2 zero-crossings per cycle
          float freqHz      = (durationSec > 0.0f) ? (cycles / durationSec) : 0.0f;

          logShakeEvent(shakeStartMs, nowMs, avgIntensity, freqHz);
        }

        isShaking         = false;
        shakeIntensitySum = 0.0f;
        shakeSampleCount  = 0;
        lastShakeEndMs    = nowMs;

        // reset frequency state between shakes
        zeroCrossCount = 0;
        lastAxisValid  = false;
        // Serial.println("Shake END");
      }
    }
  }
}

void logShakeEvent(unsigned long startMs, unsigned long endMs,
                   float avgIntensity, float freqHz) {
  unsigned long durationMs = endMs - startMs;

  // FILTER 1: ignore events that are too gentle overall
  if (avgIntensity < MIN_AVG_INTENSITY_G) {
    return;
  }

  // FILTER 2: ignore slow movements (frequency too low)
  if (freqHz < MIN_SHAKE_FREQ_HZ) {
    return;
  }

  shakeEventIndex++;
  shakesThisSession++;   // count per-session

  String line;
  line += String(shakeEventIndex);
  line += ",";
  line += String(startMs);
  line += ",";
  line += String(endMs);
  line += ",";
  line += String(durationMs);
  line += ",";
  line += String(avgIntensity, 4);
  line += ",";
  line += String(freqHz, 2);
  line += "\n";

  Serial.print("Logging shake ");
  Serial.print(shakeEventIndex);
  Serial.print(" | start = ");
  Serial.print(startMs);
  Serial.print(" ms, duration = ");
  Serial.print(durationMs);
  Serial.print(" ms, avgIntensity = ");
  Serial.print(avgIntensity, 4);
  Serial.print(", freqHz = ");
  Serial.println(freqHz, 2);

  appendLineToFile(sessionFilePath, line);
}

void appendLineToFile(const char *path, const String &line) {
  File file = SPIFFS.open(path, FILE_APPEND);
  if (!file) {
    Serial.print("Failed to open log file for appending: ");
    Serial.println(path);
    return;
  }
  file.print(line);
  file.close();
}

// ========================= BUTTON + SESSION IMPLEMENTATION =========================

void updateButton() {
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != buttonState) {
      buttonState = reading;

      // Active LOW: press detected
      if (buttonState == LOW) {
        toggleSession();
      }
    }
  }

  lastButtonReading = reading;
}

void toggleSession() {
  if (!sessionActive) {
    startSession();
  } else {
    endSession();
  }
}

void startSession() {
  sessionActive      = true;
  shakesThisSession  = 0;

  // Build unique session filename
  snprintf(sessionFilePath, sizeof(sessionFilePath),
           "/session_%03d.txt", sessionIndex++);

  // Create/truncate file and write header
  File f = SPIFFS.open(sessionFilePath, FILE_WRITE);
  if (!f) {
    Serial.print("Failed to open session file for writing: ");
    Serial.println(sessionFilePath);
    sessionActive = false;
    return;
  }

  unsigned long startMs = millis();
  Serial.print("Session started. Logging to ");
  Serial.println(sessionFilePath);

  f.println("event_index,start_ms,end_ms,duration_ms,avg_intensity_g,freq_hz");
  f.print("# Session start (ms since boot): ");
  f.println(startMs);
  f.print("# Baseline gravity (g): ");
  f.println(baselineGravityG, 4);
  f.println();
  f.close();

  // LED ON while session is active
  digitalWrite(LED_PIN, HIGH);
}

void endSession() {
  sessionActive = false;

  Serial.print("Session ended. Shakes in this session: ");
  Serial.println(shakesThisSession);
  Serial.print("Session file: ");
  Serial.println(sessionFilePath);

  // LED OFF when no active session
  digitalWrite(LED_PIN, LOW);

  // Reset shake state so a partial shake does not leak into next session
  isShaking         = false;
  shakeIntensitySum = 0.0f;
  shakeSampleCount  = 0;
  zeroCrossCount    = 0;
  lastAxisValid     = false;
}


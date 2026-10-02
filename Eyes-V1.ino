#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <esp_task_wdt.h>

// If this line fails to compile: your ESP32 Arduino core is 3.x, where the watchdog
// init signature changed. Swap the two calls in setup() for:
//   esp_task_wdt_config_t wdtConfig = { .timeout_ms = WDT_TIMEOUT_S * 1000, .idle_core_mask = 0, .trigger_panic = true };
//   esp_task_wdt_init(&wdtConfig);
//   esp_task_wdt_add(NULL);
#define WDT_TIMEOUT_S 5   // no loop feed for this long -> assume hung -> auto-reboot

// ===================== CONFIG =====================
#define DATA_PIN        18
#define NUM_LEDS        256
#define MATRIX_SIZE     16
#define SERPENTINE       1

// ---- The moving "eye" shape: now VERTICAL. 6 columns wide, up to 13 rows tall. ----
// Outer columns are shorter (12) to fake rounded left/right edges of the oval.
const uint8_t SHAPE_WIDTH = 6;
const uint8_t COL_HEIGHT[SHAPE_WIDTH] = {12, 13, 13, 13, 13, 12};
const uint8_t MAX_COL_HEIGHT = 13;

const int pupilW = 2;
const int pupilH = 3;

#define BRIGHTNESS_MIN  6
#define BRIGHTNESS_MAX  35     // lowered from 60 - was brighter than intended on a 256-LED panel
#define LDR_PIN         34     // 3-pin LDR module: + -> 3.3V, - -> GND, OUT -> this pin (NOT + or -)
#define MPU_ADDR        0x68

// ===================== STATE =====================
Adafruit_NeoPixel matrix = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

// eyeX/eyeY = top-left of the shape's bounding box, float (kept fractional on purpose - see renderFrame)
float eyeX, eyeY;
float targetX, targetY;
float easeFactor = 0.08;

const float EYE_X_MIN = 0;
const float EYE_X_MAX = MATRIX_SIZE - SHAPE_WIDTH;     // 10 - plenty of side-to-side travel
const float EYE_Y_MIN = 0;
const float EYE_Y_MAX = MATRIX_SIZE - MAX_COL_HEIGHT;  // 3  - shape is tall, little vertical room

unsigned long lastBlinkTime = 0;
unsigned long nextBlinkInterval = 3000;
bool pendingDoubleBlink = false;

// Blink now sweeps the FULL 16-row matrix, independent of where the eye currently is.
const int BLINK_PAIRS = MATRIX_SIZE / 2;        // 8 row-pairs, edge to center
const int CLOSE_STEPS = BLINK_PAIRS - 1;        // 7 pairs go fully black; center pair reserved for the dash
const int STEP_PAIR_INCREMENT = 2;              // close 2 row-pairs per step -> fewer, punchier jumps
const int NUM_CLOSE_STEPS = (CLOSE_STEPS + STEP_PAIR_INCREMENT - 1) / STEP_PAIR_INCREMENT; // 4
const int HOLD_STEP = NUM_CLOSE_STEPS + 1;
const int TOTAL_STEPS = NUM_CLOSE_STEPS * 2 + 1;

int blinkStep = 0;
unsigned long blinkStepStart = 0;
const int blinkStepMs = 25;   // full close+open now ~ (4*2-1)*25 + 60 ≈ 235ms - real blink speed
const int blinkHoldMs = 60;

float smoothedBrightness = 30;
float ldrRunningMin = 4095, ldrRunningMax = 0; // auto-ranges to your actual room's dark/bright extremes

// Idle jitter: drifts toward a slow-changing target instead of jumping every frame (kills AA flicker)
float jitterTargetX = 0, jitterTargetY = 0;
unsigned long lastJitterUpdate = 0;
const int jitterUpdateIntervalMs = 500;

// IMU calibration: whatever tilt the board reads at boot becomes "centered" (like phone auto-orientation)
float calibAX = 0, calibAY = 0, calibAZ = 0;

// ---- IMU axis mapping: TUNE THESE by watching Serial output while tilting the helmet ----
// Which raw axis feeds horizontal (X) / vertical (Y) eye movement: 0=ax, 1=ay, 2=az.
#define AXIS_X_SOURCE   1      // ay drives horizontal
#define AXIS_X_INVERT   -1     // real-eye behavior: tilt head left -> eye moves right. Flip to +1 if backwards.
#define AXIS_Y_SOURCE   2      // az drives vertical
#define AXIS_Y_INVERT   1

unsigned long lastDebugPrint = 0;

// ---- IMU failsafe state (simplified) ----
// No retry timer / center-snapping - that was causing sluggish tracking on flaky
// breadboard I2C. If a read fails, just reuse the last known-good values below,
// so an isolated dropped read is invisible and a fully-dead IMU holds its last gaze
// instead of resetting or hanging.
float lastGoodAX = 0, lastGoodAY = 0, lastGoodAZ = 0;
bool imuBootOK = false; // set once at boot; used only to decide whether to trust calibration

// ===================== SHAPE HELPERS =====================
void getColBounds(int col, int &startRow, int &height) {
  height = COL_HEIGHT[col];
  startRow = (MAX_COL_HEIGHT - height) / 2;
}

// Length of overlap between unit pixel [pxStart, pxStart+1) and float range [rStart, rEnd). Returns 0..1.
float overlap1D(float pxStart, float rStart, float rEnd) {
  float lo = max(pxStart, rStart);
  float hi = min(pxStart + 1.0f, rEnd);
  float len = hi - lo;
  if (len < 0) return 0;
  if (len > 1) return 1;
  return len;
}

bool isDashLit(int x, int eyeXr) {
  int localX = x - eyeXr;
  return (localX >= 0 && localX <= 1) || (localX >= 4 && localX <= 5);
}

// ===================== IMU =====================
void imuInit() {
  Wire.begin(21, 22);
  Wire.setTimeOut(50); // ms - stops a stuck I2C bus from blocking the loop indefinitely
                       // (if this line doesn't compile, your core doesn't support it -
                       // safe to delete; the watchdog still catches a true hang.)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

// Reads WHO_AM_I. don't check the exact value
// (varies slightly across MPU6500 revisions) - a clean response of any kind means
// the chip is wired correctly and answering on the bus.
bool imuVerify() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, true) != 1) return false;
  Wire.read();
  return true;
}

// Now returns success/failure instead of trusting whatever bytes came back.
bool imuReadAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, true) != 6) return false;

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  ax = rawX / 16384.0;
  ay = rawY / 16384.0;
  az = rawZ / 16384.0;
  return true;
}

// ===================== PIXEL MAPPING =====================
int XY(int x, int y) {
  if (SERPENTINE && (y % 2 == 1)) {
    x = MATRIX_SIZE - 1 - x;
  }
  return y * MATRIX_SIZE + x;
}

// ===================== RENDER =====================
void renderFrame() {
  int eyeXr = (int)floor(eyeX); // used only for dash placement + nearest-column lookup

  // ---- Blink state -> how many row-pairs (from top/bottom edge of the WHOLE matrix) are closed ----
  int closedPairs = 0;
  bool showDash = false;

  if (blinkStep >= 1 && blinkStep <= NUM_CLOSE_STEPS) {
    closedPairs = min(blinkStep * STEP_PAIR_INCREMENT, CLOSE_STEPS);
  } else if (blinkStep == HOLD_STEP) {
    closedPairs = CLOSE_STEPS;
    showDash = true;
  } else if (blinkStep > HOLD_STEP && blinkStep <= TOTAL_STEPS) {
    int openStepIdx = blinkStep - HOLD_STEP;      // 1..NUM_CLOSE_STEPS
    closedPairs = max(CLOSE_STEPS - openStepIdx * STEP_PAIR_INCREMENT, 0);
  }

  float pupilLeft = eyeX + (SHAPE_WIDTH - pupilW) / 2.0;
  float pupilTop  = eyeY + (MAX_COL_HEIGHT - pupilH) / 2.0;

  for (int y = 0; y < MATRIX_SIZE; y++) {
    int distFromEdge = min(y, MATRIX_SIZE - 1 - y);   // 0 (outer) .. 7 (innermost)
    bool rowClosed = distFromEdge < closedPairs;
    bool rowIsInnermost = (distFromEdge == CLOSE_STEPS);

    for (int x = 0; x < MATRIX_SIZE; x++) {
      uint8_t r, g, b;

      if (rowClosed) {
        // Eyelid: full matrix width, solid black, except a yellow sliver at the very last row-pair
        if (rowIsInnermost && showDash && isDashLit(x, eyeXr)) {
          r = 255; g = 160; b = 0;
        } else {
          r = g = b = 0;
        }
      } else {
        // ---- Antialiased black-shape coverage ----
        float hCov = overlap1D((float)x, eyeX, eyeX + SHAPE_WIDTH);
        float coverage = 0;
        if (hCov > 0) {
          int col = constrain(x - eyeXr, 0, SHAPE_WIDTH - 1); // nearest logical column
          int colStart, colHeight;
          getColBounds(col, colStart, colHeight);
          float colTop = eyeY + colStart;
          float colBottom = colTop + colHeight;
          float vCov = overlap1D((float)y, colTop, colBottom);
          coverage = hCov * vCov;
        }

        // ---- Antialiased white pupil coverage, blended on top ----
        float pHCov = overlap1D((float)x, pupilLeft, pupilLeft + pupilW);
        float pVCov = overlap1D((float)y, pupilTop, pupilTop + pupilH);
        float pupilCoverage = pHCov * pVCov;

        float baseR = 255, baseG = 160, baseB = 0;
        float rBlack = baseR * (1 - coverage);
        float gBlack = baseG * (1 - coverage);
        float bBlack = baseB * (1 - coverage);

        r = (uint8_t)round(rBlack * (1 - pupilCoverage) + 255 * pupilCoverage);
        g = (uint8_t)round(gBlack * (1 - pupilCoverage) + 255 * pupilCoverage);
        b = (uint8_t)round(bBlack * (1 - pupilCoverage) + 255 * pupilCoverage);
      }

      matrix.setPixelColor(XY(x, y), matrix.Color(r, g, b));
    }
  }

  matrix.show();
}

// ===================== BLINK STATE MACHINE =====================
void updateBlink() {
  unsigned long now = millis();

  if (blinkStep == 0) {
    if (now - lastBlinkTime > nextBlinkInterval) {
      blinkStep = 1;
      blinkStepStart = now;
      pendingDoubleBlink = (random(0, 10) == 0);
    }
    return;
  }

  unsigned long stepDuration = (blinkStep == HOLD_STEP) ? blinkHoldMs : blinkStepMs;

  if (now - blinkStepStart > stepDuration) {
    blinkStep++;
    blinkStepStart = now;

    if (blinkStep > TOTAL_STEPS) {
      blinkStep = 0;
      lastBlinkTime = now;
      if (pendingDoubleBlink) {
        pendingDoubleBlink = false;
        nextBlinkInterval = 200;
      } else {
        nextBlinkInterval = random(2000, 6000);
      }
    }
  }
}

// ===================== IMU CALIBRATION (boot-time "auto-orientation") =====================
// Samples the IMU for ~1s and averages it. Whatever tilt that is becomes the new
// zero-point, so however the helmet happens to be angled at power-on reads as "looking straight ahead."
bool calibrateIMU() {
  Serial.println("Calibrating IMU... hold the helmet still at its normal resting angle.");

  const int samples = 100;
  const int minGoodSamples = 50; // need at least half good, or the average isn't trustworthy
  float sumAX = 0, sumAY = 0, sumAZ = 0;
  int good = 0;

  for (int i = 0; i < samples; i++) {
    float ax, ay, az;
    if (imuReadAccel(ax, ay, az)) {
      sumAX += ax;
      sumAY += ay;
      sumAZ += az;
      good++;
    }
    delay(10);
  }

  if (good < minGoodSamples) {
    Serial.print("Calibration failed - only ");
    Serial.print(good);
    Serial.println("/100 good IMU reads. Starting in centered safe-state instead.");
    return false;
  }

  calibAX = sumAX / good;
  calibAY = sumAY / good;
  calibAZ = sumAZ / good;

  Serial.print("Calibration done (");
  Serial.print(good);
  Serial.print("/100 good reads). ax=");
  Serial.print(calibAX);
  Serial.print(" ay=");
  Serial.print(calibAY);
  Serial.print(" az=");
  Serial.println(calibAZ);
  return true;
}

// Picks ax/ay/az by index (0/1/2) - lets AXIS_X_SOURCE / AXIS_Y_SOURCE select any physical axis.
float selectAxis(float ax, float ay, float az, int source) {
  if (source == 0) return ax;
  if (source == 1) return ay;
  return az;
}

// ===================== IMU -> TARGET =====================
void updateTargetFromIMU() {
  unsigned long now = millis();

  float ax, ay, az;
  if (imuReadAccel(ax, ay, az)) {
    lastGoodAX = ax; lastGoodAY = ay; lastGoodAZ = az;
  } else {
    // Dropped read (breadboard flakiness, etc.) - just reuse the last known-good
    // values this frame instead of resetting anything. No snapping, no retry timer.
    ax = lastGoodAX; ay = lastGoodAY; az = lastGoodAZ;
  }

  // Occasional raw-value printout to help you pick the right AXIS_X_SOURCE / AXIS_Y_SOURCE above.
  // Tilt left/right and up/down and watch which number actually moves for which motion.
  if (now - lastDebugPrint > 500) {
    Serial.print("ax="); Serial.print(ax, 2);
    Serial.print(" ay="); Serial.print(ay, 2);
    Serial.print(" az="); Serial.println(az, 2);
    lastDebugPrint = now;
  }

  float rawXAxis = selectAxis(ax, ay, az, AXIS_X_SOURCE);
  float rawYAxis = selectAxis(ax, ay, az, AXIS_Y_SOURCE);
  float calibXAxis = selectAxis(calibAX, calibAY, calibAZ, AXIS_X_SOURCE);
  float calibYAxis = selectAxis(calibAX, calibAY, calibAZ, AXIS_Y_SOURCE);

  float relX = (rawXAxis - calibXAxis) * AXIS_X_INVERT;
  float relY = (rawYAxis - calibYAxis) * AXIS_Y_INVERT;

  float sensitivity = 1.4;
  float centerX = (EYE_X_MIN + EYE_X_MAX) / 2.0;
  float centerY = (EYE_Y_MIN + EYE_Y_MAX) / 2.0;

  float rawTX = centerX + (relX * sensitivity * (EYE_X_MAX - EYE_X_MIN) / 2.0);
  float rawTY = centerY + (relY * sensitivity * (EYE_Y_MAX - EYE_Y_MIN) / 2.0);

  targetX = constrain(rawTX, EYE_X_MIN, EYE_X_MAX);
  targetY = constrain(rawTY, EYE_Y_MIN, EYE_Y_MAX);
}

// ===================== EASING + IDLE JITTER =====================
void updateEyePosition() {
  unsigned long now = millis();

  // Idle jitter: only refresh the jitter TARGET occasionally; eyeX/Y ease toward it
  // smoothly like any other target, so there's no per-frame noise for the AA to react to.
  if (now - lastJitterUpdate > jitterUpdateIntervalMs) {
    jitterTargetX = random(-100, 101) / 100.0 * 0.4;
    jitterTargetY = random(-100, 101) / 100.0 * 0.4;
    lastJitterUpdate = now;
  }

  float dist = abs(targetX - eyeX) + abs(targetY - eyeY);
  float effectiveTargetX = targetX;
  float effectiveTargetY = targetY;
  if (dist < 0.5) {
    effectiveTargetX += jitterTargetX;
    effectiveTargetY += jitterTargetY;
  }

  eyeX += (effectiveTargetX - eyeX) * easeFactor;
  eyeY += (effectiveTargetY - eyeY) * easeFactor;

  eyeX = constrain(eyeX, EYE_X_MIN, EYE_X_MAX);
  eyeY = constrain(eyeY, EYE_Y_MIN, EYE_Y_MAX);
}

// ===================== AMBIENT BRIGHTNESS =====================
void updateBrightness() {
  int raw = analogRead(LDR_PIN);

  // Auto-range: track the darkest/brightest raw values actually seen, instead of assuming
  // the full 0-4095 ADC span. New extremes are adopted quickly-but-not-instantly (0.2 rate)
  // so one noisy/spiky reading can't permanently skew the calibration - it has to persist
  // across several frames to actually move the range.
  if (raw < ldrRunningMin) {
    ldrRunningMin += (raw - ldrRunningMin) * 0.2;
  } else {
    ldrRunningMin += (raw - ldrRunningMin) * 0.001; // slow drift, lets it re-adapt over time
  }
  if (raw > ldrRunningMax) {
    ldrRunningMax += (raw - ldrRunningMax) * 0.2;
  } else {
    ldrRunningMax += (raw - ldrRunningMax) * 0.001;
  }

  float range = max(ldrRunningMax - ldrRunningMin, 50.0f); // guard against divide-by-near-zero early on
  float norm = constrain((raw - ldrRunningMin) / range, 0.0f, 1.0f); // 0 = darkest seen, 1 = brightest seen

  // Inverted on purpose: darker room -> brighter eyes, brighter room -> dimmer eyes.
  float targetBrightness = BRIGHTNESS_MAX - norm * (BRIGHTNESS_MAX - BRIGHTNESS_MIN);

  smoothedBrightness += (targetBrightness - smoothedBrightness) * 0.02;
  matrix.setBrightness((uint8_t)smoothedBrightness);
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  matrix.begin();
  matrix.setBrightness(BRIGHTNESS_MIN);
  matrix.show();

  imuInit();

  // Boot safe-state: don't trust calibration if the IMU isn't actually answering.
  // If either check fails, we just start centered with calib left at 0 - normal
  // per-frame reads still run every loop after this (see updateTargetFromIMU), this
  // only guards against averaging garbage into the calibration at boot.
  imuBootOK = imuVerify() && calibrateIMU();
  if (!imuBootOK) {
    Serial.println("Starting centered - IMU didn't respond cleanly at boot calibration.");
  }
  imuReadAccel(lastGoodAX, lastGoodAY, lastGoodAZ); // seed so an early dropped read has something sane to fall back on

  eyeX = targetX = (EYE_X_MIN + EYE_X_MAX) / 2.0;
  eyeY = targetY = (EYE_Y_MIN + EYE_Y_MAX) / 2.0;

  randomSeed(analogRead(0));
  nextBlinkInterval = random(2000, 6000);

  // Watchdog: if loop() ever stops feeding this (stuck bus, driver lockup, whatever),
  // the ESP32 force-reboots after WDT_TIMEOUT_S instead of hanging dead at a con.
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  Serial.println("Eye v10: testing Z axis for horizontal, Y for vertical.");
}

void loop() {
  esp_task_wdt_reset(); // feed the watchdog - must happen every loop or we reboot

  updateTargetFromIMU();
  updateEyePosition();
  updateBlink();
  updateBrightness();
  renderFrame();

  delay(20);
}

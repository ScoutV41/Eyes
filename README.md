# Eyes - ESP32 LED Matrix Eye Firmware

A complete eye animation system for a WIP coslpay. Features head-tracking via IMU, responsive blinks, idle micro-movements, and adaptive brightness. Designed to be robust at conventions (watchdog auto-reboot, graceful I2C failsafe, ambient light sensing).

---

## Hardware Setup

### Pinout

| Component | ESP32 Pin | Notes |
|-----------|-----------|-------|
| WS2812B DATA | GPIO 18 | LED matrix data signal (single line drives both eyes in this breadboard version) |
| MPU6500 SDA | GPIO 21 | I2C data line to IMU |
| MPU6500 SCL | GPIO 22 | I2C clock line to IMU |
| LDR (light sensor) | GPIO 34 | Analog input for ambient brightness adjustment |

### Wiring Checklist

- **ESP32 → LED Matrix**: GPIO 18 to DATA pin (signal only; power/GND bypassed directly from wall adapter per electronics decisions doc)
- **ESP32 → MPU6500**: SDA/SCL on GPIO 21/22, 3.3V power, common GND
- **ESP32 → LDR module**: 3-pin module has `+` → 3.3V, `-` → GND, `OUT` → GPIO 34
- **Power**: Two independent sources during breadboarding
  - ESP32 USB-C for low-current logic
  - Wall adapter (5V/6A) directly to LED matrix 5V/GND + bulk capacitor (1000µF), with GND shared to ESP32

### Libraries Required

```cpp
#include <Adafruit_NeoPixel.h>   // FastLED alternative; handles WS2812B timing via RMT
#include <Wire.h>                 // I2C for MPU6500
#include <esp_task_wdt.h>         // ESP32 watchdog timer
```

Install via Arduino IDE: **Sketch → Include Library → Manage Libraries** → search for "Adafruit NeoPixel" (official, reliable).

---

## Configuration (Top of File)

All tunable parameters live in the `CONFIG` section. Defaults are production-ready for Halloween con use.

### Display Hardware

```cpp
#define DATA_PIN        18        // GPIO pin driving the LED matrix
#define NUM_LEDS        256       // 16x16 matrix (don't change unless using different panel)
#define MATRIX_SIZE     16        // Grid width/height in pixels
#define SERPENTINE      1         // 1 = zigzag pixel indexing (row 0 left-to-right, row 1 right-to-left, etc.)
```

**Serpentine note**: If your LED matrix is wired in a line (not zigzag), set to `0` and adjust `XY()` function accordingly. BTF-Lighting standard 16x16 modules are serpentine.

### Eye Shape (Oval Geometry)

```cpp
const uint8_t SHAPE_WIDTH = 6;                            // Pupil/iris area width (in pixels)
const uint8_t COL_HEIGHT[SHAPE_WIDTH] = {12, 13, 13, 13, 13, 12};  // Height per column (rounded edges)
const uint8_t MAX_COL_HEIGHT = 13;                        // Tallest column (for centering logic)
```

The 6-wide × 13-tall vertical oval creates a clean Oval eye. Columns at the edges (0, 5) are 12 pixels to fake rounded left/right edges. Adjust these arrays to reshape the eye (e.g., make it wider, taller, different aspect ratio).

### Pupil Size

```cpp
const int pupilW = 2;   // Pupil width (pixels)
const int pupilH = 3;   // Pupil height (pixels)
```

White highlight + black pupil are rendered as separate layers during each frame.

### Brightness Control

```cpp
#define BRIGHTNESS_MIN  6          // Dimmest level in dark room (don't go below 5 or LEDs go invisible)
#define BRIGHTNESS_MAX  35         // Brightest level in bright room (reduced from 60 due to 256-LED panel wattage)
#define LDR_PIN         34         // Analog pin connected to light sensor output
```

LDR auto-ranges across the min/max you actually see in your environment (dark bedroom → bright convention floor). No manual calibration needed; it learns on the fly.

### IMU Calibration & Axes

```cpp
#define MPU_ADDR        0x68       // I2C address of MPU6500 (default if AD0 left floating)
#define AXIS_X_SOURCE   1          // Which accel axis drives horizontal (X) eye motion: 0=ax, 1=ay, 2=az
#define AXIS_X_INVERT   -1         // Flip sign: -1 = tilt head left → eye moves right (normal). +1 = reversed.
#define AXIS_Y_SOURCE   2          // Which accel axis drives vertical (Y) eye motion
#define AXIS_Y_INVERT   1          // Flip sign if needed
```

**Axis tuning**: YOpen Serial Monitor (115200 baud), tilt your head left/right/up/down, watch the `ax / ay / az` values printed every 500ms, and note which axis moves for which head motion. Update `AXIS_X_SOURCE` / `AXIS_Y_SOURCE` to match. Invert flags if the eye moves backward.

### Movement Tuning

```cpp
float easeFactor = 0.08;    // Easing strength for pupil following head tilt (0.01 = sluggish, 0.2+ = snappy)
```

Lower values = smoother, more "chasing" feel. Higher values = twitchier, more direct. **0.08 is tuned for natural eye-following at con distances.**

### Blink Timing

```cpp
const int blinkStepMs = 25;    // Time per animation frame during blink (25ms = full blink ~235ms total)
const int blinkHoldMs = 60;    // Time the eye stays "fully closed" with the yellow dash visible
```

Blink speed is naturally human-like (not robotic). Occasional (~1 in 10) double-blinks add realism.

### Idle Jitter

```cpp
const int jitterUpdateIntervalMs = 500;    // How often jitter target refreshes (500ms = slow, subtle drift)
```

When the eye is roughly centered (distance < 0.5 from target), adds tiny random ±1–2 pixel movements every 500ms. Kills the "frozen" dead-eyed look even when you're standing still. Disable by setting interval to something huge (e.g., 60000).

### Watchdog (Auto-Reboot)

```cpp
#define WDT_TIMEOUT_S 5    // Reboot if loop() stops feeding watchdog for 5 seconds
```

Con failsafe: if I2C hangs, driver crashes, or anything locks up the main loop, the ESP32 force-reboots cleanly after 5 seconds instead of hanging with dead eyes. Increase if the main loop legitimately needs >5s for some reason (unlikely).

---

## Features & Behavior

### Head Tracking (IMU → Pupil Position)

1. **Boot calibration**: At power-on, IMU samples for ~1 second while you hold the helmet still at its normal resting angle. Whatever tilt is read becomes the "center" point (like phone auto-orientation).
2. **Real-time tracking**: Every loop, IMU accel is read, relative tilt is calculated, and pupil target position is updated.
3. **Easing**: Pupil doesn't snap — it smoothly "chases" toward the target using exponential smoothing (`easeFactor`), creating a natural, lived-in gaze.
4. **Edge clamping**: If head rotation would push the pupil out of the matrix bounds, it clamps to the edge and holds there, then eases back to center when you stop tilting. Reads intelligent, not cross-eyed or buggy.
5. **Failsafe**: If an I2C read from the IMU fails (breadboard flakiness), the code reuses the last good acceleration values instead of resetting or hanging. Isolated dropped reads are invisible; a fully dead IMU just holds its last gaze instead of resetting.

### Blinking

- **Frequency**: Random interval every 2–6 seconds (randomized so it doesn't feel metronomic).
- **Animation**: Sweeps in from the outer edges of the 16×16 matrix toward the center over ~7 steps (~175ms close, ~60ms hold, ~175ms open = ~235ms total, human-like speed).
- **The blink dash**: At the moment of full closure, a thin yellow horizontal line briefly appears across the center of the matrix (the "eyelash"). Reads dramatic and intentional.
- **Double-blinks**: ~1 in 10 blinks is a double (close→open→close→open again after a short pause), adding expressiveness.
- **Independence**: Blink animation sweeps the full 16-row matrix regardless of where the pupil currently is.

### Idle Micro-Movement (Jitter)

When the eye is roughly centered (within ~0.5 pixels of target), a subtle random drift is added every 500ms:
- ±0.4 pixels in X and Y
- Smooth easing toward the jitter target (not frame-to-frame noise)
- Kills the dead, frozen-doll stare even when standing still
- Completely invisible at con distances; pure "alive" vibe

### Ambient Light Adaptation

The LDR sensor on GPIO 34 auto-adjusts LED brightness to the environment:
- **Bright convention floor**: LEDs dim to avoid glare and wash-out
- **Dark cosplay room/photoshoot**: LEDs brighten for visibility and impact
- **Auto-ranging**: The code tracks the darkest and brightest raw sensor values you encounter and re-calibrates dynamically. No manual tuning needed; it learns your actual environment on startup.
- **Inverted logic**: Darker room → brighter eyes (natural instinct, and practical for low-light photos)

### Watchdog Timer (Convention Failsafe)

The ESP32 watchdog is armed at startup and must be fed every loop cycle. If the main loop hangs or deadlocks for >5 seconds, the watchdog force-reboots the board cleanly. No more "dead eyes mid-con" — just a quick 2-3 second power-on reset and you're live again.

---

## Serial Debug Output

Open **Serial Monitor at 115200 baud** to see:

- **Boot**: Calibration status, number of good IMU reads out of 100, and the calibration baseline (ax/ay/az values)
- **Running**: Every 500ms, raw accelerometer values print so you can verify which axis maps to which head motion
- **Errors**: Boot calibration failures, watchdog status, etc.

Example output:
```
Calibrating IMU... hold the IMU still at its normal resting angle.
Calibration done (98/100 good reads). ax=0.52 ay=-0.08 az=0.84
ax=0.51 ay=-0.08 az=0.83
ax=0.53 ay=-0.07 az=0.85
...
```

---

## Troubleshooting

### Eyes Not Turning On

1. **Check USB connection** to ESP32 (power supply for logic)
2. **Check LED matrix power** (separate wall adapter 5V/6A directly to matrix, capacitor in place)
3. **Verify GPIO 18** wired to matrix DATA pin
4. **Check `SERPENTINE` and `MATRIX_SIZE`** match your actual panel (should be 1 and 16 for standard BTF-Lighting)
5. **Compile and upload** the sketch; watch Serial Monitor for startup messages

### Eyes Turn On But Show Random Colors (Garbled Data)

- **I2C bus is flaky** (breadboard connections loose or bad): reseat all jumpers, especially SDA/SCL
- **LED matrix wiring**: DATA pin is super sensitive to noise; keep it short and away from power lines
- **Watchdog firing** (loop hangup then reboot): check Serial Monitor for any watchdog resets; if constant, there's a lockup somewhere

### Pupil Doesn't Track Head Motion

1. **Check IMU wiring** (SDA on GPIO 21, SCL on GPIO 22, both have pull-ups on the MPU6500 breakout)
2. **Run axis debug output**: Open Serial, tilt head in all directions, watch `ax / ay / az` values
   - If they don't move with your head tilts, IMU isn't wired right
   - If they move but the wrong axis is selected, adjust `AXIS_X_SOURCE` / `AXIS_Y_SOURCE`
3. **Axis direction backwards?** Flip the corresponding `AXIS_*_INVERT` flag (+1 ↔ -1)
4. **IMU I2C hanging**: If Serial stops printing after boot, breadboard I2C connection is flaky (reseat all jumpers)

### Eyes Too Bright or Too Dim in Rooms You Know

The LDR auto-ranges, so:
- **First startup in a new room**: It learns the dark/bright extremes over ~1–2 minutes. Give it time.
- **Persistent wrong brightness**: Check LDR wiring (OUT pin to GPIO 34, NOT + or - directly)
- **Manual override**: Adjust `BRIGHTNESS_MIN` / `BRIGHTNESS_MAX` values and re-upload (but auto-ranging should handle most cases)

### Blink Doesn't Look Right

- **Too fast/slow**: Adjust `blinkStepMs` (25ms default) or `blinkHoldMs` (60ms default)
- **Blink animation covers pupil incorrectly**: The blink sweeps the full 16-row matrix; this is intentional. If it looks wrong, check that `SERPENTINE` matches your matrix wiring.

### Pupil Jitters or Looks Twitchy

- **Lower `easeFactor`** (0.08 → 0.05) for smoother following
- **Check breadboard I2C**: flaky connections can cause erratic accelerometer readings
- **Idle jitter too aggressive?** Reduce it by increasing `jitterUpdateIntervalMs` (500 → 1000)

### ESP32 Keeps Rebooting

- **Watchdog firing**: Means the main loop is hanging (I2C deadlock, driver issue, etc.). Check Serial Monitor for any error messages.
- **Most likely cause**: I2C bus stuck low (flaky breadboard connections, loose jumper)
- **Fix**: Reseat all I2C jumpers. If still broken, add `Wire.setTimeOut(50)` in `imuInit()` (already in the code) to time out stuck bus reads instead of hanging forever

### Compiling Fails

- **`esp_task_wdt_init` error**: Your ESP32 Arduino core is version 3.x (newer). Comment out the v2.x init lines and use the v3.x init code shown at the top of the file.
- **Missing FastLED / Adafruit NeoPixel**: Install via Arduino IDE Library Manager
- **Wire.h not found**: Should be built-in; if not, update your ESP32 core

---

## Notes & Known Quirks

### Antialiasing in Motion

The pupil rendering uses subpixel-accurate overlap calculations (`overlap1D` function) so the edge stays smooth even when the pupil position is fractional (e.g., eyeX = 3.45). This is intentional; gives a much more organic, non-pixelated gaze compared to integer-only coordinates.

### LDR Auto-Ranging Tuning

The brightness auto-range uses two different adaptation rates:
- **Fast (0.2)** when a new extreme is encountered (dark area → brightness jumps up)
- **Slow (0.001)** otherwise (prevents one spike from permanently skewing the range)

This means the first time you take the helmet into a very bright room, brightness might overshoot, then settle within a few seconds. Intended behavior to avoid stale calibration.

### Idle Jitter Only Activates When "Close"

Jitter only adds movement when the pupil is near its target (`dist < 0.5`). During active head tracking, it's disabled so fast head turns read smooth and intentional, not nervous. Re-enables ~0.5 seconds after your head stops moving.

### Both Eyes on One Matrix (Future)

Current code outputs to a single 16x16 matrix. For true dual-eye setup, you'd duplicate the rendering logic, send two independent DATA streams to GPIO 18 and another pin via RMT, and render both eyes side by side or on separate matrices. The animation/IMU/blink logic would be shared (both eyes always look the same direction).

### Watchdog on GPIO

The watchdog is fed every single loop iteration. If you add blocking operations (long `delay()`, I2C hangs, etc.), the watchdog will reboot. The 5-second timeout is conservative; normal loop runtime is <<5ms, so you have tons of buffer.

---

## Flashing Instructions

1. **Install Arduino IDE** and the **ESP32 board package** (use official Arduino → Preferences → Add board manager URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`)
2. **Install Adafruit NeoPixel library** (Sketch → Include Library → Manage Libraries → search "NeoPixel" → install by Adafruit)
3. **Connect ESP32 via USB-C** to your computer
4. **Select board**: Tools → Board → esp32 → "ESP32 Dev Module"
5. **Select port**: Tools → Port → (your COM/ttyUSB port)
6. **Open EyesV1ino** in Arduino IDE
7. **Verify**: Sketch → Verify (compiles, checks for errors)
8. **Upload**: Sketch → Upload (compiles + flashes to board)
9. **Open Serial Monitor** (Tools → Serial Monitor, 115200 baud) to see startup messages

---

## Version History

- **v1**: Final breadboard version with antialiased rendering, robust I2C failsafe, LDR auto-ranging, watchdog timer, and realistic blink timing. Production-ready for Halloween.
- v0: Iterative prototyping (axis mapping, blink shape refinement, etc.)

---

## Future Improvements 

- Dual independent WS2812B outputs (separate eye matrices)
- Mood/expression system (angry, sad, excited animations on command)
- Manual trigger button (glove-mounted, wired to GPIO for "blink on demand" performance beat)
- Integration with costume audio (eyes react to dialogue/sound effects)

---

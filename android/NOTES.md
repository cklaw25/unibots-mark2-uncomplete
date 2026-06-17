# Mark 4 Ping Pong Robot — Project Notes
Last updated: 2026-06-15

## Project Structure
- **Android app:** `C:\mark 4 ping pong\android_dylan_rough\` (this repo)
- **ESP32 code:** `C:\esp32\src\main.cpp` (PlatformIO project, flash from here)
- **ESP32 source in repo:** `esp32_main.cpp` (keep in sync with C:\esp32\src\main.cpp)

---

## Architecture
- Phone (Android, YOLO11/OpenCV via ncnn + AprilTag via apriltag C library) detects ping pong ball and AprilTags
- **Primary transport: USB serial (OTG USB-C to USB-C cable, 115200 baud)** — stable, no WiFi needed
- **Fallback: WiFi UDP** — ESP32 still runs AP (`PingPong_ESP32`, `12345678`), UDP port 4210, IP `192.168.4.1`
- ESP32 listens on both paths simultaneously
- Task mode **5** = ball tracking (Part 1), Task mode **6** = AprilTag navigation (Part 2)

---

## Auto-Cycling (Java, MainActivity.java)
- **Part 1 (Task 5):** runs 90 seconds → auto-switches to Task 6
- **Part 2 (Task 6):** runs 90 seconds → auto-switches back to Task 5 (OR immediately on delivery complete)
- Cycle repeats indefinitely
- Timers cancel on app pause, reset on task load
- Both timers displayed on screen
- **`pollDeliveryComplete` Runnable** polls `yolo11ncnn.checkAndClearPart1Switch()` every 500ms while in Task 6 — switches immediately to Task 5 when delivery is done

---

## Packet Formats

### Ball packet (Part 1, Android → ESP32)
```
frame_seq, timestamp_ms, frame_w, frame_h, num_targets, has_primary,
bx, by, bw, bh, prob, is_centered, err_x, err_y [, ...more objects]
```
Ends with `\n`.

### Command packet (Part 2, Android → ESP32)
```
CMD,<command>\n
```
Commands: `STOP`, `ROTATE_CW`, `ROTATE_CCW`, `FORWARD`, `BACKWARD`, `LIFT_UP`, `LIFT_DOWN`

ESP32 detects `CMD,` prefix to distinguish from ball packets.

---

## Motor Pin Mapping (ESP32)
| GPIO | L298N | Motor |
|------|-------|-------|
| 26 | #1 IN1 | Back Left |
| 25 | #1 IN2 | Back Left |
| 33 | #1 IN3 | Front Left |
| 32 | #1 IN4 | Front Left |
| 14 | #2 IN1 | Back Right |
| 27 | #2 IN2 | Back Right |
| 23 | #2 IN3 | Front Right |
| 22 | #2 IN4 | Front Right |
| 19 | #3 IN1 | Lift UP |
| 18 | #3 IN2 | Lift DOWN |
| 15 | IN3 | Spinner |
| 21 | IN4 | Spinner |

**Power:** Two battery packs in series (~24V), LM2596 buck converter → ESP32 VIN

---

## Current Working State

### Android (`yolo11ncnn.cpp`)

#### Part 1 — Ball tracking (Task 5)
- Centre rectangle: orientation-aware, `CENTER_RECT_HEIGHT=120`
- USB serial JNI bridge: C++ calls `YOLO11Ncnn.sendPacketUsb()` via stored JavaVM refs on every frame
- Mirrored robot FSM on-screen: ROTATING / MOVING FORWARD / COASTING (2500ms)
- LED flash x3 trigger mirrored on screen
- **Part 1 countdown timer shown on screen** (`P1 TIME: Xs remaining`)
- `g_part1_reset_requested` flag resets timer when Task 5 is loaded

#### Part 2 — AprilTag navigation (Task 6)
Full FSM (11 states):

```
P2_SCAN         → rotate CW for SCAN_STEP_MS (~45 deg) → P2_SCAN_PAUSE
P2_SCAN_PAUSE   → stop still for SCAN_PAUSE_MS (1500ms), look for tags
                  tag found → P2_ALIGN
                  timeout → P2_SCAN (another step)
P2_ALIGN        → tag found, rotate left/right until tag centre within +/-40px of screen centre
                  NOTE: err_x > 0 → ROTATE_CCW, err_x < 0 → ROTATE_CW (physically correct)
P2_APPROACH     → tag centred, move FORWARD
                  non-target tag: stop when size >= 150px → P2_FIND_NEXT
                  target tag (1): stop when size >= 70px → P2_TURN_180
P2_FIND_NEXT    → arrived at non-target tag, rotate shortest path toward Tag 1
                  new tag visible → P2_ALIGN; safety timeout 10s → P2_SCAN
P2_TURN_180     → rotate CW for 1500ms → P2_REVERSE
P2_REVERSE      → move BACKWARD for 2s → P2_LIFT_UP
P2_LIFT_UP      → LIFT_UP for 3s → P2_LIFT_DOWN
P2_LIFT_DOWN    → LIFT_DOWN for 1s → P2_FORWARD_EXIT
P2_FORWARD_EXIT → move FORWARD for 2s → STOP → set g_request_part1_switch=true → P2_DONE
P2_DONE         → STOP, Java switches to Part 1 immediately via poll
```

**Constants (tunable in `yolo11ncnn.cpp`):**
- `TARGET_TAG = 1`
- `NUM_TAGS = 24`
- `ALIGN_THRESH = 40` px
- `CLOSE_ENOUGH = 150.0f` px (tag side length, non-target)
- `TARGET_RANGE_PX = 70.0f` px (trigger delivery for target tag)
- `TURN_180_MS = 1500.0` ms
- `PHASE_SECS = 90.0`
- `SCAN_STEP_MS = 400.0` ms (~45 degree rotation step)
- `SCAN_PAUSE_MS = 1500.0` ms (pause to look for tags)

### ESP32 (`esp32_main.cpp` / `C:\esp32\src\main.cpp`)

#### Part 1 FSM — Step-scan rotation
```
STATE_STEP_ROTATE  → rotate CW for ROTATE_STEP_MS (400ms ~45 deg) → STATE_SCAN_PAUSE
STATE_SCAN_PAUSE   → stop still for SCAN_PAUSE_MS (1500ms)
                     ball in centre → flash LED x3, move forward → STATE_MOVING_FORWARD
                     ball visible but off-centre → align pulse → STATE_ALIGN_ROTATE
                     timeout → STATE_STEP_ROTATE
STATE_ALIGN_ROTATE → short rotation pulse (200ms) toward ball → STATE_SCAN_PAUSE
                     NOTE: err_x > 0 → set_rotate_ccw(), err_x < 0 → set_rotate()
STATE_MOVING_FORWARD → drive forward while ball is centred
STATE_COASTING       → 2500ms coast, then spinner sequence → STATE_STEP_ROTATE
```

**Constants (tunable in `main.cpp`):**
- `ROTATE_STEP_MS = 400` ms (~45 degrees, tune)
- `SCAN_PAUSE_MS = 1500` ms
- `ALIGN_STEP_MS = 200` ms per alignment pulse

#### Part 2 mode
- `g_part2_active` flag: set on first CMD packet, cleared on first ball packet
- `execute_p2_command()` handles: `STOP`, `ROTATE_CW`, `ROTATE_CCW`, `FORWARD`, `BACKWARD`, `LIFT_UP`, `LIFT_DOWN`
- `stop_all_motors()` also zeros spinner pins
- When first ball packet arrives after Part 2 → resets to `STATE_STEP_ROTATE`

#### Spinner motor (GPIO 15 = IN3, GPIO 21 = IN4)
- Fires at end of Part 1 coasting cycle (before returning to step-rotate)
- Sequence: CW 300ms → stop → wait 300ms → CCW 300ms → stop
- `SPINNER_DEG120_MS = 300` — tune for actual 120° rotation

---

## Rotation Direction Notes (IMPORTANT)
- **`set_rotate()` = CW physically** (left motors forward, right backward)
- **`set_rotate_ccw()` = CCW physically** (left motors backward, right forward)
- **Alignment logic is INVERTED from naive expectation:**
  - `err_x > 0` (object to the RIGHT) → use `set_rotate_ccw()` / send `ROTATE_CCW`
  - `err_x < 0` (object to the LEFT) → use `set_rotate()` / send `ROTATE_CW`
  - This was verified physically and fixed

---

## Camera Notes
- **Front camera:** better for ball detection (YOLO11) — wider FOV, closer range
- **Back camera:** only camera that reliably detects AprilTags — sharper optics, higher resolution
- Must **manually switch camera** between Part 1 (front) and Part 2 (back)

---

## Key Build Settings
- `gradle.properties`: `android.useAndroidX=true`
- `app/build.gradle`: `implementation 'com.github.mik3y:usb-serial-for-android:3.7.0'`
- `settings.gradle`: JitPack added
- `CMakeLists.txt`: FetchContent for apriltag (GIT_TAG master), cmake_minimum 3.11

---

## Known Issues / Tuning Notes
- **LEDC PWM broke rotation** — do not use LEDC, use digitalWrite + software duty cycle
- **move_forward() uses `false`** for all motors (direction physically reversed)
- **ROTATE_STEP_MS = 400** — tune for actual ~45 degree step
- **CLOSE_ENOUGH = 150px** — tune based on tag physical size and camera distance
- **SPINNER_DEG120_MS = 300** — tune based on actual spinner motor speed
- **User has Tags 0 and 1 printed** — Tag 1 is target
- **Downscaling Task 6 frame corrupted the display** — do not resize rgb in draw_apriltag_mode()

---

## Test Results
- Ball detection and tracking ✓
- USB serial connection stable ✓
- LED flashes 3x when ball enters centre ✓
- Robot moves forward correctly ✓
- Robot coasts 2.5s after ball leaves centre ✓
- Step-scan rotation (Part 1) ✓
- Alignment direction fix (Part 1 + Part 2) ✓
- Part 2: can find tag and drive forward to it ✓
- Part 2: full tag-to-tag navigation — NOT YET TESTED
- Auto-switch Part 1 → Part 2 (Java timer) ✓
- Spinner after ball tracking cycle — NOT YET TESTED
- Full delivery sequence (Tag 1 at 70px → 180° → reverse → lift → forward → Part 1) — NOT YET TESTED

---

## To Flash ESP32
```bash
cd C:\esp32
pio run --target upload
```

## To Build & Install Android App
```bash
cd "C:\mark 4 ping pong\android_dylan_rough"
./gradlew installDebug
```
(Phone plugged into PC via USB with ADB authorised — disconnect OTG cable first)

# Session Conversation Log & Project Handoff
Last saved: 2026-06-21 (Session 9 — complete, pushed to GitHub)

This file is a human-readable summary of the full development session so far.
It is intended to allow someone picking this up on a new machine to understand
exactly where we left off, what decisions were made and why, and what to do next.

---

## Session 9 — 2026-06-21 (Two-board rewrite — BoardA + BoardB + Android)

### What changed
- New schematic reviewed: `pin codes details/extra stuff since 216/schematic_two_board.md`
- **TT4 GPIO1/3 conflict is FIXED** — new pin map avoids GPIO1/3 entirely
- **Stepper motor dropped** — replaced by N20 #2 (lift motor on Board B)
- **Architecture: two ESP32 boards confirmed**
  - Board A = wheels + MPU6050 + phone USB serial
  - Board B = N20 spinner + N20 lift + encoders
- **GPIO4 wire protocol** (Board A output, Board B input):
  - LOW sustained = Mode 1 → Board B spins spinner
  - LOW→HIGH = Mode 2 start → Board B rotates 90° gate, holds (balls can't slip)
  - HIGH→LOW 200ms→HIGH = deposit pulse → Board B lifts, holds, lowers
  - HIGH→LOW sustained = Mode 1 resuming → Board B opens gate, resumes spinner
- **Bug fixed**: original UnibotsMainV2 never set `currentMode = MODE_2`, causing
  `driveForwardHold()` to exit early during Mode 2 navigation. Fixed in BoardA.ino.

### New files written
| File | Purpose |
|------|---------|
| `arduino/BoardA/BoardA.ino` | Complete Board A firmware (new pins, GPIO4, no spinner) |
| `arduino/BoardA/platformio.ini` | PlatformIO config for COM7 |
| `arduino/BoardB/BoardB.ino` | Complete Board B firmware (spinner + lift + encoder) |
| `arduino/BoardB/platformio.ini` | PlatformIO config — ⚠️ update COMX to Board B's port |
| `android/.../MainActivity.java` | Removed auto-switch, auto-connect USB, auto-select Task 5 |

### Complete new pin maps

**Board A (wheels ESP32, COM7):**
| Motor | PWM | IN1 | IN2 | Invert |
|-------|-----|-----|-----|--------|
| TT1 front-right | GPIO13 | GPIO14 | GPIO16 | false |
| TT2 front-left  | GPIO17 | GPIO18 | GPIO19 | true (guess) |
| TT3 back-right  | GPIO23 | GPIO25 | GPIO26 | false |
| TT4 back-left   | GPIO27 | GPIO32 | GPIO33 | true (guess) |
| MPU6050 SDA | GPIO21 | | | |
| MPU6050 SCL | GPIO22 | | | |
| Trigger OUT | GPIO4 | | | |

**Board B (N20 ESP32, COMX):**
| Component | PWM | IN1 | IN2 | ENC C1 | ENC C2 |
|-----------|-----|-----|-----|--------|--------|
| N20 #1 spinner | GPIO13 | GPIO14 | GPIO27 | GPIO32 | GPIO33 |
| N20 #2 lift    | GPIO16 | GPIO17 | GPIO18 | (unused) | (unused) |
| Trigger IN | GPIO4 | | | | |

### What teammates need to do after flashing
1. **Board A: invert flags** — if any wheel drives backward, flip its `invert` in BoardA.ino
2. **Board A: MS_PER_MM** — drive 500mm, time it in ms, set `MS_PER_MM = ms/500`
3. **Board B: SPINNER_PULSES_REV** — send `m` in Serial Monitor, spin N20 shaft 1 revolution by hand, read delta count, set constant
4. **Board B: LIFT_UP_MS / LIFT_DOWN_MS** — send `l` in Serial Monitor, watch lift extend/retract, tune timing
5. **Board B: platformio.ini** — replace `COMX` with actual COM port of Board B ESP32

### Additional updates (same session)
- `BoardA.ino` — added 5-second boot window to enter motor test mode
  - `1`-`4` = test individual wheel, `a` = all forward, `d` = 5s drive for MS_PER_MM calibration, `q` = quit to match
- `android/main.xml` — added live status TextView (`textStatusUsb`)
- `android/MainActivity.java` — status label polls every 2s: "ESP32: Connected" (green) / "ESP32: Not connected" (red)
- `README.md` — complete rewrite: from-scratch setup guide (Step 1-7), per-component test procedures, tuning constants table, full troubleshooting table (15 entries)

### State at close
- **All code pushed to GitHub** — commit `c59d25e` on master
- README has full teammate-facing guide: fresh computer → running match (Steps 1-7)
- Calibration still needed on-site (teammates do these after flashing):
  1. Invert flags — test each wheel with `1`-`4`, fix any backward wheels
  2. MS_PER_MM — use `d` command, measure 5s drive distance, set = 5000/mm
  3. SPINNER_PULSES_REV — use `m` command on Board B, spin shaft 1 rev by hand
  4. LIFT_UP_MS / LIFT_DOWN_MS — use `l` command on Board B, tune until full travel
  5. Board B COM port — update `COMX` in `arduino/BoardB/platformio.ini`

---

## Session 8 — 2026-06-17 (GitHub update + README usage guide)

### What happened
- Added "How to use the code" section to README covering UnibotsMainV2 and HardwareTest
  - UnibotsMainV2: boot sequence, Serial Monitor output format, tuning constants table
  - HardwareTest: command reference (1-5, a, m, p, o, s), recommended bring-up order
- Pushed everything to GitHub (`cklaw25/unibots-mark2-uncomplete`):
  - `arduino/UnibotsMainV2/` — new combined match firmware
  - `arduino/HardwareTest/` — bench test tool
  - `android/` — full YOLO11 Android app with model weights
  - `README.md` — rewritten for new user/Claude sessions
  - `CONVERSATION_LOG.md` — updated
- No code changes this session

### State at close
- GitHub fully up to date
- Robot is ready to be physically brought up — outstanding items before match:
  1. TT4 re-wire: GPIO1/3 → GPIO18/19
  2. Stepper motor pins confirmed and filled into UnibotsMainV2.ino
  3. MS_PER_MM calibrated after first flash
  4. N20 encoder pins verified with HardwareTest `m` command

---

## Session 7 — 2026-06-17 (UnibotsMainV2 generated)

### What happened
- Clarified that "broken 2-motor scheme" referred to the CODE structure in UnibotsMain.ino (only 2 PIN defines for left/right groups), NOT the hardware — all 4 wheels were already confirmed working
- Clarified TT4 TX/RX conflict: TT4's direction pins on GPIO1(TX)/GPIO3(RX) block Android serial packets whenever TT4 is driven — nothing to do with N20
- **Hardware fix decided:** re-wire TT4 IN1/IN2 from GPIO1/GPIO3 → GPIO18/GPIO19 (2-wire re-route, frees UART0 fully)
- Confirmed N20 spinner activation timeline: turns ON at end of ALIGN (ball centred), stays ON through APPROACH and COAST, turns OFF when COAST ends. Never on during Mode 2.
- Stepper motor type/pins TBD — stubs left in code
- Generated `arduino/UnibotsMainV2/UnibotsMainV2.ino` — full combined firmware

### Key code changes vs UnibotsMain.ino
- Replaced 2-motor PIN scheme with 4-motor `MotorPins` struct (from HardwareTest), invert flags included
- TT4 IN1/IN2 now GPIO18/GPIO19 (re-wired)
- Fixed APPROACH and COAST heading-hold bug: original code had `normalizeAngle(heading_deg - heading_deg)` (always 0) — fixed by capturing `approachHeading` / `coastHeading` on state entry
- Stepper stubs replace linear actuator (STEPPER_STEP_PIN=0 guard so it no-ops safely until pins confirmed)
- No collision sensor anywhere in code
- STBY not touched (hardwired HIGH)

### State at close
- `UnibotsMainV2.ino` written and ready to flash
- **Before flashing:** do the TT4 GPIO1/3 → GPIO18/19 re-wire
- **After flashing:** confirm N20 encoder with `m` command (still use HardwareTest for bench checks), calibrate MS_PER_MM, fill in stepper pins once driver confirmed

---

## Session 6 — 2026-06-17 (Component list confirmed)

### What happened
- Confirmed final component list for the robot
- **Collision sensor DROPPED** — will not be used
- **Linear actuator REPLACED by stepper motor** for ball deposit mechanism
- No code changes this session

### Confirmed component list
| Component | Details |
|-----------|---------|
| ESP32-D0WD-V3 (WROOM-32) | Brain, COM7 |
| Android phone | YOLO11 vision, USB serial |
| 4× TT DC motors | Drive, see pin map |
| 3× TB6612FNG | Motor drivers, STBY hardwired HIGH |
| N20 motor w/ encoder | Spinner wheel, encoder unconfirmed |
| MPU6050 | Gyro/IMU, SDA=21 SCL=22 |
| Stepper motor | Ball deposit (replaces linear actuator) |

### State at close
- Component list finalised
- Next step: port confirmed pin map into UnibotsMain.ino and restructure for 4 independent TT motors + stepper

---

## Session 5 — 2026-06-17 (Catch-up / log update session)

### What happened
- Reopened project after a context-limit cut-off in Session 4
- User pasted the lost Session 4 conversation as `C:\unibotsmark2\text file for pincode testing 166.txt`
- Updated `CONVERSATION_LOG.md` with full Session 4 details (hardware bring-up, pin map, motor direction fixes)
- User confirmed: **all 4 TT wheels drive forward together correctly** and **N20 confirmed spinning** — this wasn't recorded before
- Updated memory and conversation log with that final confirmation
- No code changes this session

### State at close
- All 5 motors working, direction correct, straight-line drive confirmed
- `HardwareTest.ino` is the current flashed firmware on COM7
- **Next step:** port confirmed pin/invert map into `UnibotsMain.ino` (the real match firmware)

---

## Session 4 — 2026-06-16 (Hardware bring-up — all motors confirmed working)

### What happened
- Created `arduino/HardwareTest/HardwareTest.ino` — interactive Serial Monitor test script (1/2/3/4/5/a/m/p/o/s commands)
- Created `arduino/HardwareTest/platformio.ini` — board=esp32dev, port=COM7, speed=921600
- Chip confirmed as **ESP32-D0WD-V3** (classic dual-core WROOM-32), NOT S3 — update any code/docs that said S3
- Flashed and verified via `pio run -t upload --upload-port COM7` from `C:\unibotsmark2\arduino\HardwareTest\`
- Found and fixed direction issues on TT2 (front-left) and TT4 (back-left) — both needed `invert=true`
- TT3 was temporarily inverted then reverted (don't assume same-axle motors share a direction fault)
- Front wheels (TT1/TT2) initially appeared dead — turned out to be a hardware power/wiring issue, not pin codes
- All 5 motors now confirmed spinning correctly

### Confirmed pin map (from pin codes details/ photos — improvised wiring)

| Motor | PWM | IN1 | IN2 | Inverted? | Notes |
|-------|-----|-----|-----|-----------|-------|
| TT1 front-right | GPIO14 | GPIO13 | GPIO23 | no | |
| TT2 front-left | GPIO16 | GPIO17 | GPIO5 | **yes** | |
| TT3 back-right | GPIO4 | GPIO15 | GPIO2 | no | |
| TT4 back-left | GPIO12 | GPIO1 (TX) | GPIO3 (RX) | **yes** | Shares USB Serial pins — Serial drops during TT4 test, recovers after |
| N20 | GPIO25 | GPIO27 | GPIO26 | no | |

### N20 encoder (unconfirmed)
- Best guess from diagram: C1=GPIO32, C2=GPIO33
- Use `m` command (spin shaft by hand, 8s monitor) to confirm — if zero pulses, edit `N20_ENC_C1/C2` defines and reflash
- Encoder interrupt wired on RISING edge of C1, checks C2 for direction

### Other key hardware notes
- STBY on all 3 TB6612 chips is hardwired to 3.3V (always enabled) — do NOT treat as a GPIO
- All 3 TB6612 chips need separate motor-supply voltage (VM) AND logic VCC
- TT4's TX/RX conflict: script handles it by only claiming those pins during the '4' and 'a' tests, then restores Serial.begin() afterward

### HardwareTest.ino commands
| Key | Action |
|-----|--------|
| 1-5 | Test that motor (forward 600ms → stop → reverse 600ms → stop) |
| a | All 4 TT motors forward together 1.2s |
| m | Monitor N20 encoder for 8s while hand-spinning shaft |
| p | Probe TT1/TT2 input pins one at a time (hold HIGH 3s — use multimeter) |
| o | Drive TT1 then TT2 forward 6s each (multimeter the A01/A02, B01/B02 output terminals) |
| s | Stop everything |

### State at close
- All 5 motors working, direction correct
- **All 4 TT wheels drive forward together correctly** (confirmed via `a` command — robot would roll straight)
- **N20 confirmed spinning** under motor power
- Next step: port confirmed pin/invert map into `UnibotsMain.ino` (currently only handles 2 drive motors, not 4 independent ones)

---

## Session 3 — 2026-06-15 (Quick test run)

### What happened
- Ran the Python simulation (`simulation/robot_sim.py`) — completed successfully, 9mm error on return to start, pure dead-reckoning, no tag correction needed
- Ran the PC vision test (`pc_test/vision_test.py`) — loaded YOLOv8 model + camera (1280×720) successfully. Target classes: 32=sports ball, 47=apple, 49=orange. Run it directly in terminal (not background) so the window stays open: `cd C:\unibotsmark2\pc_test && python vision_test.py`
- No code changes this session

### State at close
- Everything is working as expected
- Next step: provide hardware pin numbers → fill in `UnibotsMain.ino` → flash to Elegoo V4 → test on real robot

---

## Session 2 — 2026-06-15 (Git setup + clarification session)

### What happened
- Confirmed UnibotsMarkII (`C:\unibotsmark2\`) is the **one and only active project** going forward
- Mark 4 Ping Pong (`C:\mark 4 ping pong\android_dylan_rough\`) is reference only — do not write to it
- Created a clean GitHub repo: `cklaw25/unibots-mark2-uncomplete`
- Pushed all UnibotsMarkII code (arduino, esp32, simulation, pc_test, rulebook screenshots)
- Wrote new README.md and this CONVERSATION_LOG.md
- **Key correction:** odometry is the permanent Mode 2 return path. It will NOT be replaced by AprilTag visual navigation when migrating to Android+ESP32. Odometry is the core upgrade over Mark 4.

### Project folder structure confirmed
| Path | Purpose |
|------|---------|
| `C:\unibotsmark2\` | Main active project (this repo) |
| `C:\unibotsmark2_backup_2026-06-13\` | Backup — do not edit |
| `C:\mark 4 ping pong\android_dylan_rough\` | Old project — reference only |
| `C:\esp32\` | Old ESP32 firmware — reference only |

### State at close of session
- Everything committed and pushed to `cklaw25/unibots-mark2-uncomplete`
- No code changes made — this was purely a git/docs session
- Next session: provide pin numbers and start hardware testing (see "What to do next" section below)

---

## What this project is

**UnibotsMarkII** — an autonomous competition robot for Unibots UK 2025-2026.

Current brain: Elegoo V4 (ESP32-based) + Arduino framework.
Future plan: Migrate to Android phone + ESP32 (same approach as Mark 4 Ping Pong project).

The robot:
1. **Mode 1 (60s):** Scans for orange ping-pong balls using YOLO11 vision (Android phone sends data over USB serial), drives toward them, collects with spinner wheel
2. **Mode 2:** Uses odometry (dead-reckoning + gyro) to navigate back to start, deposits balls with linear actuator, then restarts Mode 1

---

## Where the code lives

| Component | Path |
|-----------|------|
| Main robot sketch | `arduino/UnibotsMain/UnibotsMain.ino` |
| Python simulation | `simulation/` |
| Motor test sketches | `arduino/MotorControl/`, `arduino/MotorTest/` |
| ESP32 return test | `esp32/ReturnTest/` |

---

## Development session summary

### What was built
- **Mode 1 FSM**: step-scan rotation → align toward ball → drive forward (heading-hold) → coast → repeat
- **Mode 2 FSM**: compute bearing to start (atan2 of odometry) → turn to face home → drive home (time-based) → fine approach pulses → deposit (actuator up) → retract → backup → reset odometry
- **Gyro integration**: MPU6050 read directly over I2C (no library), calibrated at startup, integrated into heading_deg
- **Odometry**: tracks x/y in mm from start, updated every COAST phase, used in Mode 2 for return path
- **Python simulation**: full 2D sim with noise modelling, tag-correction mid-return, wall-seek final phase

### What was tested
- Gyro heading-hold while driving — **WORKING on real hardware**
- Mode 2 return-to-start (gyro + odometry) — **WORKING on real hardware**
- Mode 1 ball scan and approach — **WORKING**
- Full Mode 1 → Mode 2 → Mode 1 cycle — **WORKING** (but actuator/spinner pins are placeholders)

### What is NOT done yet
- **UnibotsMain.ino still has placeholder pins** — needs updating with the confirmed map from Session 4
- Spinner motor (N20) encoder pins unconfirmed (GPIO32/33 best guess — use `m` command to verify)
- Linear actuator not wired/tested
- Collision sensor not wired/tested

---

## Current progress

| Feature | Status |
|---------|--------|
| Mode 1 ball collection FSM | Working |
| Gyro heading-hold | Working on real hardware |
| Mode 2 return-to-start (odometry + gyro) | Working on real hardware |
| Python simulation | Working |
| Linear actuator (deposit) | Code ready, pins are placeholders |
| Spinner motor | Code ready, pins are placeholders |
| Collision sensor | Code ready, pin is placeholder |
| 4 TT drive motors confirmed working | Done — Session 4 |
| N20 motor confirmed working | Done — encoder unconfirmed |
| UnibotsMain.ino updated with real pins | NOT YET — next task |
| Full end-to-end with all hardware | NOT YET |

---

## What to do when you pick this up on a new machine

### 1. Clone this repo
```bash
git clone https://github.com/cklaw25/unibots-mark2-uncomplete.git
cd unibots-mark2-uncomplete
```

### 2. Fill in the pin numbers
Open `arduino/UnibotsMain/UnibotsMain.ino` and fill in the PIN definitions at the top.
All are currently set to `0` as placeholders:
```cpp
#define PIN_PWMA        __  // Right motor speed (PWM)
#define PIN_PWMB        __  // Left motor speed (PWM)
#define PIN_AIN         __  // Right motor direction
#define PIN_BIN         __  // Left motor direction
#define PIN_STBY        __  // Motor enable
#define PIN_SPINNER_1   __  // Spinner IN1
#define PIN_SPINNER_2   __  // Spinner IN2
#define PIN_ACTUATOR_UP __  // Linear actuator extend
#define PIN_ACTUATOR_DOWN __ // Linear actuator retract
#define PIN_COLLISION   __  // Collision sensor
```

Forbidden ESP32-S3 pins (do not use): GPIO 19, 20 (USB), 35/36/37 (PSRAM), 22-25 (don't exist on S3).

### 3. Calibrate MS_PER_MM
Run the robot forward exactly 500mm on a flat surface, time it in milliseconds.
Set `MS_PER_MM = measured_ms / 500` in `UnibotsMain.ino`.
This is critical for accurate return-to-start navigation.

### 4. Flash
- Open `arduino/UnibotsMain/UnibotsMain.ino` in Arduino IDE or PlatformIO
- Select board: Elegoo V4 (or any ESP32-based board)
- Upload

### 5. Test sequence (recommended order)
1. Test motors spin the right direction (use `MotorTest` sketch first)
2. Test gyro calibrates and heading-hold works (Serial monitor — watch heading_deg)
3. Test Mode 1 alone — ball detection + drive
4. Test Mode 2 alone — manually trigger return path
5. Full end-to-end cycle

---

## Critical reminders for next session

1. **UnibotsMain.ino still has placeholder pins.** Real pin map is now known (confirmed in Session 4 — see Session 4 table above). Update `UnibotsMain.ino` before a full match-firmware test. `HardwareTest.ino` already has the correct confirmed map.

2. **Calibrate MS_PER_MM first.** The return-to-start distance is entirely based on this. Wrong value = robot drives past home or stops short.

3. **Keep robot still during startup.** The gyro calibrates for ~1 second at boot. Any movement during this window corrupts the heading baseline.

4. **Rotation direction is inverted from naive expectation:**
   `err_x > 0` (ball on RIGHT) → rotate CCW
   `err_x < 0` (ball on LEFT) → rotate CW
   This is verified correct. Do not change it.

5. **analogWrite() requires ESP32 Arduino core 3.x.** If using an older core, swap to `ledcWrite()`.

6. **GYRO_SIGN = 1** — if the robot turns the wrong way when heading-correcting, flip this to -1.

---

## Future migration plan

The end goal is to replace the Elegoo V4 / Arduino brain with:
- **Android phone** for vision (YOLO11 ball detection, AprilTag navigation)
- **ESP32** as motor controller receiving commands from the phone

When this happens:
- Mode 1 stays largely the same (phone already sends ball packets over USB serial — same format)
- Mode 2 **keeps odometry as the primary return-path method** — this is the core upgrade over Mark 4 and is a permanent design decision, not a stepping stone
- The simulation folder stays fully relevant — it directly models the odometry return path

The odometry dead-reckoning is what Mark 4 Ping Pong lacked. It is the main reason UnibotsMarkII exists as a separate project. Do not replace it with AprilTag visual navigation.

---

## Python Simulation Notes

The simulation (`simulation/`) is useful for testing the return-path logic without hardware:
- `odometry.py` — dead-reckoning (integrates forward/rotate commands into x/y/angle)
- `return_nav.py` — return navigator (bearing → drive → fine approach → wall-seek)
- `localizer.py` — opportunistic AprilTag correction mid-return
- `robot_sim.py` — 2D visualiser

Key simulation constants (tune to match real robot):
- `FORWARD_SPEED_MM_S = 200` — adjust to match real robot speed
- `ROTATE_SPEED_DEG_S = 90` — adjust to match real robot turn speed
- `FORWARD_NOISE_FRAC = 0.02` — simulates 2% motor noise (realistic for PWM without encoders)

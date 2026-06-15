# Session Conversation Log & Project Handoff
Last saved: 2026-06-15 (updated same day — git/repo setup session)

This file is a human-readable summary of the full development session so far.
It is intended to allow someone picking this up on a new machine to understand
exactly where we left off, what decisions were made and why, and what to do next.

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
- **Pin numbers are all 0 (placeholders)** — must fill in `UnibotsMain.ino` before a full hardware test
- Spinner motor not wired/tested
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
| Full end-to-end with all hardware | NOT YET — needs pin numbers |

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

1. **All pins are placeholders (set to 0).** The robot will boot and the mode cycle will run, but nothing will move. Fill in real pin numbers before any hardware test.

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

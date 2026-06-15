# Unibots Mark II

Autonomous robot for the Unibots UK 2025-2026 competition.

**Current brain:** Elegoo V4 (ESP32-based) running Arduino framework
**Future plan:** Migrate brain to Android phone + ESP32 (same approach as Mark 4 Ping Pong project)

---

## If you are completely new to this project, read this first

**What is this robot?**
An autonomous robot that collects ping-pong balls and deposits them into a scoring net. It runs for 60 seconds collecting balls (Mode 1), then navigates back to the start position and deposits (Mode 2), then repeats.

**How does it know where it is?**
It uses **dead-reckoning odometry** — it tracks every move it makes (how far forward, how many degrees it turned) and uses that to calculate where it is relative to where it started. It also uses a **gyroscope (MPU6050)** to hold a straight heading while driving. There's no GPS, no encoders — just time-based estimates.

**Two modes:**
- **Mode 1 (60 seconds):** Scan for ball → align toward it → drive forward (spinner on) → coast → repeat
- **Mode 2:** Compute bearing and distance back to start → turn to face home → drive home → deposit → backup → restart Mode 1

**Where each thing is in this repo:**

| Folder | What it contains |
|--------|-----------------|
| `arduino/UnibotsMain/` | Main robot code (flash this to the Elegoo V4) |
| `arduino/MotorControl/` | Motor control test sketch |
| `arduino/MotorTest/` | Individual motor test |
| `esp32/ReturnTest/` | ESP32 return-path test sketch |
| `esp32/PinScanner/` | Pin scanning utility |
| `simulation/` | Python simulation for testing navigation without hardware |
| `rulebook arena dimensions/` | Competition arena specs |

---

## Current Progress (as of 2026-06-15)

| Feature | Status |
|---------|--------|
| Mode 1: ball scan-step rotation | Working |
| Mode 1: align toward ball | Working |
| Mode 1: drive forward + spinner | Working |
| Mode 1: coast after ball lost | Working |
| Gyro heading-hold while driving | Working |
| Mode 2: odometry dead-reckoning | Working |
| Mode 2: gyro return-to-start | **Working on real hardware** |
| Mode 2: deposit (linear actuator) | Placeholder — pins not set yet |
| Spinner motor | Placeholder — pins not set yet |
| All motor pins | **Placeholders (all set to 0) — fill in before flashing** |

**The biggest thing blocking a full test run: pin numbers are all placeholders (set to 0) in `UnibotsMain.ino`. Fill these in with your actual wiring before flashing.**

---

## Setup — How to get this running on a new machine

### Step 1 — Clone this repo
```bash
git clone https://github.com/cklaw25/unibots-mark2-uncomplete.git
```

### Step 2 — Flash the main sketch (Arduino)
- Install Arduino IDE or PlatformIO
- Add ESP32 board support (if using Arduino IDE: Boards Manager → search "esp32" → install Espressif)
- Open `arduino/UnibotsMain/UnibotsMain.ino`
- **Before flashing: fill in all the PIN definitions at the top of the file** (they are all set to 0 as placeholders)
- Select your board (Elegoo V4 = ESP32-based board)
- Upload

### Step 3 — Run the Python simulation (optional but useful)
```bash
pip install matplotlib numpy
cd simulation
python robot_sim.py
```
This lets you test the return-to-start navigation on screen before running on real hardware.

---

## Hardware Overview

**Brain:** Elegoo V4 (ESP32 chip, Arduino framework)

**Sensors:**
- MPU6050 gyro (I2C) — heading hold + drift correction
- Android phone (USB serial, 115200 baud) — sends ball detection packets

**Actuators:**
- Drive motors × 2 (differential drive, tank-style steering)
- Spinner motor — collects balls on contact
- Linear actuator — raises to deposit balls into net

**Pin mapping — ALL PLACEHOLDERS, fill in from your wiring:**
```cpp
PIN_PWMA         // Right motor speed (PWM)
PIN_PWMB         // Left motor speed (PWM)
PIN_AIN          // Right motor direction
PIN_BIN          // Left motor direction
PIN_STBY         // Motor enable
PIN_SPINNER_1    // Spinner IN1
PIN_SPINNER_2    // Spinner IN2
PIN_ACTUATOR_UP  // Linear actuator extend
PIN_ACTUATOR_DOWN // Linear actuator retract
PIN_COLLISION    // Collision sensor
PIN_SDA = 21     // MPU6050 I2C data
PIN_SCL = 22     // MPU6050 I2C clock
```

**Power:** TBD

---

## How the Code Works

### Mode 1 — Ball Collection FSM

```
S1_SCAN_STEP  → rotate CW for 400ms (~45 degrees)
S1_SCAN_PAUSE → stop 1.5s, wait for ball packet from Android
                ball visible → S1_ALIGN
                timeout → S1_SCAN_STEP (next step)
S1_ALIGN      → pulse-rotate 200ms toward ball
                ball centred → spinner ON → S1_APPROACH
S1_APPROACH   → drive forward (gyro heading-hold)
                ball lost for 5 consecutive frames → S1_COAST
S1_COAST      → drive forward 2s, spinner ON
                → record distance in odometry → S1_SCAN_STEP
```

Mode 1 runs for 60 seconds, then auto-switches to Mode 2.

### Mode 2 — Return to Start

```
S2_COMPUTE     → calculate bearing + distance to start using odometry
S2_NAVIGATE    → turn to face home (gyro-controlled), drive home (time-based)
                 fine approach: 4 short pulses to reach wall
S2_AT_POSITION → arrived
S2_DEPOSIT     → linear actuator UP for 1s
S2_RETRACT     → linear actuator DOWN for 0.6s
S2_BACKUP      → reverse 0.7s
                 → reset odometry → restart Mode 1
```

### Odometry (dead-reckoning)
Robot tracks its position as `(x, y)` in mm from start. Every time it drives forward, it adds `(duration_ms / MS_PER_MM)` to its position based on the current gyro heading. When it returns home, it computes `atan2(-y, -x)` for the bearing and `sqrt(x²+y²)` for the distance.

**Key constant to calibrate:**
`MS_PER_MM = 3.5` — run robot forward 500mm, time it, divide: `ms / 500 = MS_PER_MM`

### Rotation Direction Note
`err_x > 0` (ball on RIGHT of screen) → rotate **CCW**
`err_x < 0` (ball on LEFT) → rotate **CW**
This is physically correct. Do not change it.

---

## Tunable Constants (in UnibotsMain.ino)

| Constant | Value | What it controls |
|----------|-------|-----------------|
| `MS_PER_MM` | 3.5 | Distance calibration — **calibrate first** |
| `SPEED` | 150 | Forward drive speed (0-255) |
| `TURN_SPEED` | 120 | Rotation speed |
| `KP` | 1.8 | Heading-hold P-gain (reduce if oscillating) |
| `TURN_THRESH` | 3.0° | Stop turning within this many degrees |
| `MODE1_DURATION_MS` | 60000 | How long Mode 1 runs (ms) |
| `SCAN_STEP_MS` | 400 | Rotation step duration (~45°) |
| `SCAN_PAUSE_MS` | 1500 | Pause to look for ball |
| `ALIGN_STEP_MS` | 200 | Alignment pulse duration |
| `COAST_MS` | 2000 | Coast time after ball lost |
| `CENTRE_ERR_PX` | 100 | Pixel threshold for "ball is centred" |
| `BALL_LOST_FRAMES` | 5 | Frames without ball before COAST |

---

## Python Simulation

The `simulation/` folder lets you test the navigation before running on hardware:

- `odometry.py` — dead-reckoning tracker (integrates commands into x/y/angle)
- `return_nav.py` — return-to-start navigator (bearing → drive → fine approach → wall-seek)
- `localizer.py` — AprilTag-based position correction (opportunistic tag fixes mid-return)
- `robot_sim.py` — 2D visualiser
- `env_map.py` — arena dimensions and tag positions

To use it:
```bash
pip install matplotlib numpy
cd simulation
python robot_sim.py
```

Edit `TAG_POSITIONS` in `env_map.py` to match your actual field measurements.

---

## Future Plan — Migration to Android + ESP32

The current brain is the Elegoo V4 running Arduino code. The plan is to eventually migrate to:
- **Android phone** as the vision/brain (YOLO11 ball detection, AprilTag navigation)
- **ESP32** as the motor controller

This is already implemented in a separate project (Mark 4 Ping Pong). When the migration happens, the Mode 2 return path will switch from odometry-based to AprilTag visual navigation.

The serial packet format (ball detection CSV from Android) is already compatible between the two projects.

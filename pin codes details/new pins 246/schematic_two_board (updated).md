# Two-board schematic — clean, grouped, safe GPIOs

BOARD A = wheels board. Phone-powered (USB). Drives 4 wheels + reads MPU.
BOARD B = drive-motor board. LM2596-powered (5V). TWO N20 motors + encoders.

Master/timing: BOARD A is the master. It runs the wait + wheels, then
raises the trigger. BOARD B waits for the trigger, then runs its part.

## POWER RULE (unchanged, important)
- Share GROUND between both boards (and with the whole system). REQUIRED.
- NEVER tie Board A's 3V3/5V to Board B's 3V3/5V. Each board regulates its own.

## GPIO SAFETY (why these pins)
Avoided everywhere: GPIO0/2/5/12/15 (strapping), 34/35/36/39 (input-only),
6-11 (flash), 1/3 (USB serial). All pins below are safe general-purpose I/O.

# NOTE!!!! [ONLY PINS IN BOLD ARE CHANGED]
================================================================
# BOARD A — WHEELS + MPU   (powered by phone USB)
================================================================

## Power
Phone USB        -> Board A (power + data)
Board A 3V3      -> TB6612 #2 VCC
Board A 3V3      -> TB6612 #2 STBY
Board A 3V3      -> TB6612 #3 VCC
Board A 3V3      -> TB6612 #3 STBY
Board A 3V3      -> MPU-6050 VCC
Board A GND      -> Common GND rail

## TB6612 #2 — FRONT wheels   (motor power = 6V buck)
TB6612 #2 VM     -> Buck OUT+ (6V)
TB6612 #2 VCC    -> Board A 3V3
TB6612 #2 GND    -> Common GND rail
TB6612 #2 STBY   -> Board A 3V3
**TB6612 #2 PWMA   -> GPIO32**      (front right)
**TB6612 #2 AIN1   -> GPIO25**
**TB6612 #2 AIN2   -> GPIO33**
TB6612 #2 AO1    -> Front right motor wire 1
TB6612 #2 AO2    -> Front right motor wire 2
**TB6612 #2 PWMB   -> GPIO14**          (front left)
**TB6612 #2 BIN1   -> GPIO26**
**TB6612 #2 BIN2   -> GPIO27**
TB6612 #2 BO1    -> Front left motor wire 1
TB6612 #2 BO2    -> Front left motor wire 2

## TB6612 #3 — BACK wheels   (motor power = 6V buck)
TB6612 #3 VM     -> Buck OUT+ (6V)
TB6612 #3 VCC    -> Board A 3V3
TB6612 #3 GND    -> Common GND rail
TB6612 #3 STBY   -> Board A 3V3
**TB6612 #3 PWMA   -> GPIO13**          (back right)
**TB6612 #3 AIN1   -> GPIO17**
**TB6612 #3 AIN2   -> GPIO16**
TB6612 #3 AO1    -> Back right motor wire 1
TB6612 #3 AO2    -> Back right motor wire 2
**TB6612 #3 PWMB   -> GPIO23**          (back left)
**TB6612 #3 BIN1   -> GPIO18**
**TB6612 #3 BIN2   -> GPI19**
TB6612 #3 BO1    -> Back left motor wire 1
TB6612 #3 BO2    -> Back left motor wire 2

## MPU-6050   (I2C)
MPU-6050 VCC     -> Board A 3V3
MPU-6050 GND     -> Common GND rail
MPU-6050 SDA     -> GPIO21
MPU-6050 SCL     -> GPIO22

## Trigger OUT (to Board B)
Board A GPIO4    -> Board B GPIO4   (the "go" signal)

Board A GPIO map:  13 14 16 17 18 19 | 23 25 26 27 32 33 | 21 22 | 4
                   front wheels       back wheels         MPU     trigger


# NOTE!!! [BOARD B IS UNCHANGED, ONLY HIGHLIGHTED PINS IN BOARD A IS CHANGED]

================================================================
# BOARD B — TWO N20 MOTORS + ENCODERS   (powered by LM2596 5V)
================================================================

## Power
LM2596 IN+       -> 12V+
LM2596 IN-       -> 12V-  (common ground)
LM2596 OUT+ (5V) -> Board B VIN        (set LM2596 to 5.0V, MEASURE FIRST)
LM2596 OUT-      -> Common GND rail
Board B 3V3      -> TB6612 #1 VCC
Board B 3V3      -> TB6612 #1 STBY
Board B 3V3      -> N20 #1 encoder VCC
Board B 3V3      -> N20 #2 encoder VCC
Board B GND      -> Common GND rail

## TB6612 #1 — DRIVES BOTH N20 MOTORS   (motor power = 12V)
TB6612 #1 VM     -> 12V+
TB6612 #1 VCC    -> Board B 3V3
TB6612 #1 GND    -> Common GND rail
TB6612 #1 STBY   -> Board B 3V3
# --- Channel A = N20 motor #1 ---
TB6612 #1 PWMA   -> GPIO13      (left side)
TB6612 #1 AIN1   -> GPIO14      (left side)
TB6612 #1 AIN2   -> GPIO27      (left side)
TB6612 #1 AO1    -> N20 #1 wire 1
TB6612 #1 AO2    -> N20 #1 wire 2
# --- Channel B = N20 motor #2 (the lift motor) ---
TB6612 #1 PWMB   -> GPIO16      (right side)
TB6612 #1 BIN1   -> GPIO17      (right side)
TB6612 #1 BIN2   -> GPIO18      (right side)
TB6612 #1 BO1    -> N20 #2 wire 1
TB6612 #1 BO2    -> N20 #2 wire 2

## N20 ENCODER #1
N20 #1 encoder VCC  -> Board B 3V3
N20 #1 encoder GND  -> Common GND rail
N20 #1 C1           -> GPIO32      (left side, use INPUT_PULLUP in code)
N20 #1 C2           -> GPIO33      (left side, use INPUT_PULLUP in code)

## N20 ENCODER #2
N20 #2 encoder VCC  -> Board B 3V3
N20 #2 encoder GND  -> Common GND rail
N20 #2 C1           -> GPIO19      (right side, use INPUT_PULLUP in code)
N20 #2 C2           -> GPIO21      (right side, use INPUT_PULLUP in code)

## Trigger IN (from Board A)
Board B GPIO4    <- Board A GPIO4   (right side - only off-board wire here)
Board B GND      <-> Board A GND   (via the common ground rail)

Board B GPIO map:
  LEFT  side: 13 14 27 | 32 33      = N20 #1 motor + encoder #1
  RIGHT side: 16 17 18 | 19 21 | 4  = N20 #2 motor + encoder #2 + trigger
  (free: 25 26 on left, 22 23 on right)

================================================================
# SHARED RAILS  (one set, feeding both boards' motor sides)
================================================================
LiPo +  -> FUSE -> SWITCH -> 12V+ rail   (fuse right at the battery +)
LiPo -  -> 12V- (common ground)
12V+   -> TB6612 #1 VM, Buck(6V) IN+, LM2596 IN+
12V-   -> Common GND rail
Buck(6V) OUT+ -> TB6612 #2 VM, TB6612 #3 VM
Buck(6V) OUT- -> Common GND rail
  (TB6612 #1's single VM feeds BOTH N20 motors - one driver, two channels.)

## COMMON GROUND RAIL ties together:
12V- , Buck(6V) OUT- , LM2596 OUT- ,
Board A GND , Board B GND ,
all TB6612 GND , MPU GND , both N20 encoder GNDs

================================================================
# PROGRAMMING / STARTUP NOTES
================================================================
- Upload each board with its motor power (and the LM2596) OFF, so no
  two 5V sources fight and no motor twitches during flashing.
- Board A has the phone -> keep GPIO1/3 clear (they are). Don't add Serial.
- To run: power everything, prop wheels up, reset BOARD A. It runs the
  wait+wheels, then triggers Board B to run N20 #1 then N20 #2. Board B can
  be left powered and waiting; it follows Board A automatically.

// Arduino UNO — Motor Controller
// Receives commands from ESP32-CAM via hardware serial (pins 0/1)
// Command format:  F,<ms>   = forward
//                  B,<ms>   = backward
//                  R,<ms>   = rotate CW
//                  L,<ms>   = rotate CCW
//                  S        = stop
// Replies "ACK\n" when done so ESP32 knows to send the next command.
//
// LED 13 debug:
//   5 fast blinks on boot
//   2 blinks when any serial command arrives
//   LED on solid 300ms when ACK is sent

// TB6612 motor driver pins
#define STBY  3
#define PWM_R 5
#define PWM_L 6
#define DIR_R 7
#define DIR_L 8
#define LED   13

#define MOTOR_SPEED 180

void blink(int n, int ms) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED, HIGH); delay(ms);
    digitalWrite(LED, LOW);  delay(ms);
  }
}

void setup() {
  pinMode(LED,   OUTPUT);
  pinMode(STBY,  OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  digitalWrite(STBY, HIGH);
  stopMotors();
  Serial.begin(115200);
  blink(5, 80);   // boot OK signal
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;
    blink(2, 60);           // command received signal
    executeCommand(cmd);
    Serial.println("ACK");
    digitalWrite(LED, HIGH); delay(300); digitalWrite(LED, LOW);  // ACK sent
  }
}

void executeCommand(String cmd) {
  if (cmd.startsWith("F,")) {
    moveForward(cmd.substring(2).toInt());
  } else if (cmd.startsWith("B,")) {
    moveBackward(cmd.substring(2).toInt());
  } else if (cmd.startsWith("R,")) {
    rotateCW(cmd.substring(2).toInt());
  } else if (cmd.startsWith("L,")) {
    rotateCCW(cmd.substring(2).toInt());
  } else if (cmd == "S") {
    stopMotors();
  } else if (cmd == "PING") {
    blink(3, 60);   // 3 fast blinks = PING received
  }
}

void moveForward(int ms) {
  digitalWrite(DIR_R, HIGH);
  digitalWrite(DIR_L, HIGH);
  analogWrite(PWM_R, MOTOR_SPEED);
  analogWrite(PWM_L, MOTOR_SPEED);
  delay(ms);
  stopMotors();
}

void moveBackward(int ms) {
  digitalWrite(DIR_R, LOW);
  digitalWrite(DIR_L, LOW);
  analogWrite(PWM_R, MOTOR_SPEED);
  analogWrite(PWM_L, MOTOR_SPEED);
  delay(ms);
  stopMotors();
}

void rotateCW(int ms) {
  digitalWrite(DIR_R, LOW);
  digitalWrite(DIR_L, HIGH);
  analogWrite(PWM_R, MOTOR_SPEED);
  analogWrite(PWM_L, MOTOR_SPEED);
  delay(ms);
  stopMotors();
}

void rotateCCW(int ms) {
  digitalWrite(DIR_R, HIGH);
  digitalWrite(DIR_L, LOW);
  analogWrite(PWM_R, MOTOR_SPEED);
  analogWrite(PWM_L, MOTOR_SPEED);
  delay(ms);
  stopMotors();
}

void stopMotors() {
  analogWrite(PWM_R, 0);
  analogWrite(PWM_L, 0);
}

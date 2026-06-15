// TEST A ONLY: TB6612 pins — STBY=3, PWM_R=5, PWM_L=6, DIR_R=7, DIR_L=8
// 2s boot delay, then motors forward for 2s, then stop.

#define SPD 200

void setup() {
  delay(2000);

  pinMode(3, OUTPUT); digitalWrite(3, HIGH);   // STBY enable
  pinMode(7, OUTPUT); digitalWrite(7, HIGH);   // DIR_R forward
  pinMode(8, OUTPUT); digitalWrite(8, HIGH);   // DIR_L forward
  pinMode(5, OUTPUT); analogWrite(5, SPD);     // PWM_R
  pinMode(6, OUTPUT); analogWrite(6, SPD);     // PWM_L

  delay(2000);

  analogWrite(5, 0);
  analogWrite(6, 0);
}

void loop() {}

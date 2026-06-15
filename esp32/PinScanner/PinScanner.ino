#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial ArduinoSerial(2);

// GPIO19/20 = USB D-/D+ on ESP32-S3 — never use as UART
// GPIO35/36/37 = PSRAM — skip
// GPIO22-25 = don't exist on ESP32-S3
struct PinPair { int rx; int tx; };
PinPair pairs[] = {
  // Previously tried (no response) — included to confirm switch position
  {33, 4},  {4, 33},
  {16, 17}, {17, 16},
  {44, 43}, {43, 44},
  {1,  2},  {2,  1},
  {21, 22}, {22, 21},  // 22 doesn't exist on S3 but safe to try
  // New candidates
  {18, 17}, {17, 18},
  {5,  6},  {6,  5},
  {7,  8},  {8,  7},
  {9, 10},  {10,  9},
  {12, 13}, {13, 12},
  {14, 15}, {15, 14},
  {38, 39}, {39, 38},
  {40, 41}, {41, 40},
  {42, 48}, {48, 42},
  {3,  46}, {46,  3},
};
const int NUM_PAIRS = sizeof(pairs) / sizeof(pairs[0]);

bool tryPair(int rx, int tx, int baud) {
  ArduinoSerial.end();
  delay(50);
  ArduinoSerial.begin(baud, SERIAL_8N1, rx, tx);
  delay(100);
  for (int attempt = 0; attempt < 3; attempt++) {
    ArduinoSerial.println("PING");
    unsigned long t = millis();
    while (millis() - t < 500) {
      if (ArduinoSerial.available()) {
        String resp = ArduinoSerial.readStringUntil('\n');
        resp.trim();
        if (resp == "ACK") return true;
      }
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== PIN SCANNER v2 ===");

  for (int i = 0; i < NUM_PAIRS; i++) {
    int rx = pairs[i].rx;
    int tx = pairs[i].tx;

    // Try 115200 first, then 9600
    for (int baud : {115200, 9600}) {
      Serial.printf("Trying RX=%d TX=%d @ %d ...", rx, tx, baud);
      if (tryPair(rx, tx, baud)) {
        Serial.printf(" *** FOUND! RX=%d TX=%d @ %d ***\n", rx, tx, baud);
        // Keep pinging so Arduino LED stays active
        while (true) { ArduinoSerial.println("PING"); delay(800); }
      }
      Serial.println(" no");
    }
  }
  Serial.println("\nAll pairs exhausted. Check switch position.");
}

void loop() {}

#include "BluetoothSerial.h"

#define THROTTLE_PIN1 25
#define THROTTLE_PIN2 26
#define THROTTLE_PIN3 27
#define THROTTLE_PIN4 33

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_MOTOR");  // Bluetooth name
  Serial.println("Bluetooth device ready. Pair with 'ESP32_MOTOR'");

  pinMode(THROTTLE_PIN1, OUTPUT);
  pinMode(THROTTLE_PIN2, OUTPUT);
  pinMode(THROTTLE_PIN3, OUTPUT);
  pinMode(THROTTLE_PIN4, OUTPUT);
}

void loop() {
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    if (cmd == "ON") {
      int speed = 150;
      analogWrite(THROTTLE_PIN1, speed);
      analogWrite(THROTTLE_PIN2, speed);
      analogWrite(THROTTLE_PIN3, speed);
      analogWrite(THROTTLE_PIN4, speed);
      Serial.println("Motors ON at speed 90");
      SerialBT.println("Motors running at 90");
    }
    else if (cmd == "OFF") {
      analogWrite(THROTTLE_PIN1, 0);
      analogWrite(THROTTLE_PIN2, 0);
      analogWrite(THROTTLE_PIN3, 0);
      analogWrite(THROTTLE_PIN4, 0);
      Serial.println("Motors OFF");
      SerialBT.println("Motors stopped");
    }
  }
}

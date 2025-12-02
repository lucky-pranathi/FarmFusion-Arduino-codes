#include "BluetoothSerial.h"

#define THROTTLE_PIN1 25
#define THROTTLE_PIN2 26
#define THROTTLE_PIN3 27
#define THROTTLE_PIN4 33
#define REVERSE_PIN   32   // Relay control for BLDC reverse wires

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_MOTOR");  // Bluetooth name
  Serial.println("Bluetooth device ready. Pair with 'ESP32_MOTOR'");

  pinMode(THROTTLE_PIN1, OUTPUT);
  pinMode(THROTTLE_PIN2, OUTPUT);
  pinMode(THROTTLE_PIN3, OUTPUT);
  pinMode(THROTTLE_PIN4, OUTPUT);

  pinMode(REVERSE_PIN, OUTPUT);
  digitalWrite(REVERSE_PIN, LOW); // default: forward
}

void loop() {
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    if (cmd == "FORWARD") {
      digitalWrite(REVERSE_PIN, LOW);  // forward mode
      int speed = 150;
      analogWrite(THROTTLE_PIN1, speed);
      analogWrite(THROTTLE_PIN2, speed);
      analogWrite(THROTTLE_PIN3, speed);
      analogWrite(THROTTLE_PIN4, speed);
      Serial.println("Moving Forward");
      SerialBT.println("Moving Forward");
    }
    else if (cmd == "BACKWARD") {
      digitalWrite(REVERSE_PIN, HIGH); // reverse mode
      int speed = 150;
      analogWrite(THROTTLE_PIN1, speed);
      analogWrite(THROTTLE_PIN2, speed);
      analogWrite(THROTTLE_PIN3, speed);
      analogWrite(THROTTLE_PIN4, speed);
      Serial.println("Moving Backward");
      SerialBT.println("Moving Backward");
    }
    else if (cmd == "STOP") {
      analogWrite(THROTTLE_PIN1, 0);
      analogWrite(THROTTLE_PIN2, 0);
      analogWrite(THROTTLE_PIN3, 0);
      analogWrite(THROTTLE_PIN4, 0);
      Serial.println("Motors Stopped");
      SerialBT.println("Motors Stopped");
    }
  }
}

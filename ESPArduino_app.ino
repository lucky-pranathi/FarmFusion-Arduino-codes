#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// ===== HUB MOTOR =====
#define THROTTLE_PIN1 32
#define REVERSE_PIN   22
#define enable 2

volatile int revFlag = 0;
int hubSpeedPWM = 0;

// ===== UART TO ARDUINO =====
#define TX2 17
#define RX2 16

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_MOTOR");

  Serial2.begin(9600, SERIAL_8N1, RX2, TX2);

  pinMode(THROTTLE_PIN1, OUTPUT);
  pinMode(REVERSE_PIN, OUTPUT);
  digitalWrite(REVERSE_PIN, HIGH);
  pinMode(enable,OUTPUT);
  digitalWrite(enable,HIGH);
}

// ====================== HUB MOTOR ONLY ======================
void startHub(){
  if(revFlag==1){
    digitalWrite(REVERSE_PIN, LOW);
    delay(1000);
    digitalWrite(REVERSE_PIN, HIGH);
    revFlag = 0;
  }
  analogWrite(THROTTLE_PIN1, hubSpeedPWM);
}

void stopHub(){
  analogWrite(THROTTLE_PIN1, 0);
}

void reverseHub(){
  if(revFlag==0){
    digitalWrite(REVERSE_PIN, LOW);
    delay(1000);
    digitalWrite(REVERSE_PIN, HIGH);
    revFlag = 1;
  }
  analogWrite(THROTTLE_PIN1, hubSpeedPWM);
}

void setHubSpeed(int percent){
  hubSpeedPWM = map(percent, 0, 100, 0, 255);
}

// ====================== MAIN LOOP ==========================
void loop() {

  if (SerialBT.available()) {

    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    int code = -1;

    // ====== HUB CONTROL (ESP32 ONLY) ======
    if (cmd == "FORWARD"){
      startHub();
      return;
    }
    if (cmd == "STOP"){
      stopHub();
      return;
    }
    if (cmd == "BACKWARD"){
      reverseHub();
      return;
    }
    if (cmd.startsWith("HSPD:")){
      int sp = cmd.substring(5).toInt();
      if (sp >= 0 && sp <= 100)
        setHubSpeed(sp);
      return;
    }

    // ====== FORWARD REMAINING TO ARDUINO ======
    if (cmd.startsWith("SPD:")) code = 4;
    else if (cmd == "ROTATE_90") code = 5;
    else if (cmd == "ROTATE_0") code = 6;
    else if (cmd == "stop_rotate") code = 17;
    else if (cmd == "MOVE_DOWN_HYDRO") code = 7;
    else if (cmd == "MOVE_UP_HYDRO") code = 8;
    else if (cmd == "STOP_HYDRO") code = 9;
    else if (cmd == "spray_on") code = 12;
    else if (cmd == "spray_off") code = 13;
    else if (cmd == "activate_spray") code = 14;
    else if (cmd == "deactivate_spray") code = 15;
    else if (cmd == "stop_actuator") code = 16;
    else if (cmd == "SMALL_HYDRO_UP") code = 18;
    else if (cmd == "SMALL_HYDRO_STOP") code = 19;
    else if(cmd == "SMALL_HYDRO_DOWN") code= 20;

    // ===== SEND TO ARDUINO =====
    if (code == 4) {
      String value = cmd.substring(4);
      Serial2.println("4:" + value);
    }
    else if (code != -1) {
      Serial2.println(String(code));
    }
  }
}

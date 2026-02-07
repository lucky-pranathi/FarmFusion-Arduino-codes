#include <SoftwareSerial.h>

// ===== SOFTWARE SERIAL FOR ESP32 =====
SoftwareSerial ESP(7, 8);   // RX = 7, TX = 8

/* ===== Shared PWM Pins ===== */
#define RPWM 10
#define LPWM 11
#define wingsRPWM 3
#define wingsLPWM 5
#define smallHydroRPWM 6
#define smallHydroLPWM 9

/* ===== Individual ENABLE PINS ===== */
#define EN_H 2   // Hydraulic
#define EN_R 12  // Rotation
#define EN_S 4   // Spray motor
#define codeEnable 13

int spraySpeedPWM = 0;

/* ===== Disable all controllers ===== */
void disableAll() {
  digitalWrite(EN_H, LOW);
  digitalWrite(EN_R, LOW);
  digitalWrite(EN_S, LOW);
  Serial.println("[ACTION] All controllers disabled");
}

/* ===== Send PWM to selected controller ===== */
void driveMotor(int r, int l) {
  analogWrite(RPWM, r);
  analogWrite(LPWM, l);
  Serial.print("[PWM] RPWM = "); Serial.print(r);
  Serial.print(" | LPWM = "); Serial.println(l);
}

/* ===== HYDRAULIC ===== */
void hydroDown() {
  Serial.println("[CMD] HYDRAULIC DOWN");
  disableAll();
  digitalWrite(EN_H, HIGH);
  driveMotor(180, 0);
}
void hydroUp() {
  Serial.println("[CMD] HYDRAULIC UP");
  disableAll();
  digitalWrite(EN_H, HIGH);
  driveMotor(0, 180);
}
void hydroStop() {
  Serial.println("[CMD] HYDRAULIC STOP");
  disableAll();
  driveMotor(0, 0);
}

/* ===== ROTATION ===== */
void rotateRight() {
  Serial.println("[CMD] ROTATE RIGHT");
  disableAll();
  digitalWrite(EN_R, HIGH);
  driveMotor(160, 0);
}
void rotateLeft() {
  Serial.println("[CMD] ROTATE LEFT");
  disableAll();
  digitalWrite(EN_R, HIGH);
  driveMotor(0, 160);
}
void stopRotate() {
  Serial.println("[CMD] ROTATION STOP");
  disableAll();
  driveMotor(0, 0);
}

/* ===== SPRAY MOTOR ===== */
void setSpraySpeed(int percent){
  spraySpeedPWM = map(percent, 0, 100, 0, 255);
  Serial.print("[CMD] Spray Speed set to ");
  Serial.print(percent);
  Serial.print("% -> PWM = ");
  Serial.println(spraySpeedPWM);
}
void sprayOn() {
  Serial.println("[CMD] SPRAY MOTOR ON");
  disableAll();
  digitalWrite(EN_S, HIGH);
  driveMotor(spraySpeedPWM, 0);
}
void sprayOff() {
  Serial.println("[CMD] SPRAY MOTOR OFF");
  disableAll();
  driveMotor(0, 0);
}

/* ===== SPRAY ACTUATOR ===== */
void activateSpray() {
  Serial.println("[CMD] ACTUATOR EXTEND");
  analogWrite(wingsRPWM,180);
  analogWrite(wingsLPWM, 0);
}
void deactivateSpray() {
  Serial.println("[CMD] ACTUATOR RETRACT");
  analogWrite(wingsRPWM,0);
  analogWrite(wingsLPWM, 180);
}
void stopActuator() {
  Serial.println("[CMD] ACTUATOR STOP");
  analogWrite(wingsRPWM,0);
  analogWrite(wingsLPWM, 0);
}

/* ===== SMALL HYDRAULIC ===== */
void smallHydroUp(){
  Serial.println("[CMD] SMALL HYDRAULIC UP");
  analogWrite(smallHydroRPWM,0);
  analogWrite(smallHydroLPWM,180);
}

void smallHydroDown(){
  Serial.println("[CMD] SMALL HYDRAULIC DOWN");
  analogWrite(smallHydroRPWM,180);
  analogWrite(smallHydroLPWM,0);
}

void smallHydroStop(){
  Serial.println("[CMD] SMALL HYDRAULIC STOP");
  analogWrite(smallHydroRPWM,0);
  analogWrite(smallHydroLPWM,0);
}

/* ===========================================================
                     SETUP
   =========================================================== */
void setup() {

  Serial.begin(9600);
  ESP.begin(9600);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  pinMode(EN_H, OUTPUT);
  pinMode(EN_R, OUTPUT);
  pinMode(EN_S, OUTPUT);
  pinMode(codeEnable,OUTPUT);
  digitalWrite(codeEnable,HIGH);

  disableAll();
  driveMotor(0,0);

  Serial.println("\n====== ARDUINO READY (SoftwareSerial 7,8) ======\n");
}

/* ===========================================================
                     MAIN LOOP
   =========================================================== */
void loop() {

  if (ESP.available()) {

    String cmd = ESP.readStringUntil('\n');
    cmd.trim();

    Serial.print("[RAW DATA] Received: ");
    Serial.println(cmd);

    int code = -1;
    int value = -1;

    if (cmd.indexOf(':') != -1) {
      code = cmd.substring(0, cmd.indexOf(':')).toInt();
      value = cmd.substring(cmd.indexOf(':') + 1).toInt();

      Serial.print("[PARSED] Code = ");
      Serial.print(code);
      Serial.print(" | Value = ");
      Serial.println(value);
    }
    else {
      code = cmd.toInt();
      Serial.print("[PARSED] Code = ");
      Serial.println(code);
    }

    switch (code) {

      case 4:   setSpraySpeed(value); break;

      case 5:   rotateRight(); break;
      case 6:   rotateLeft(); break;
      case 17:  stopRotate(); break;

      case 7:   hydroDown(); break;
      case 8:   hydroUp(); break;
      case 9:   hydroStop(); break;

      case 12:  sprayOn(); break;
      case 13:  sprayOff(); break;

      case 14:  activateSpray(); break;
      case 15:  deactivateSpray(); break;
      case 16:  stopActuator(); break;

      case 18:  smallHydroUp(); break;
      case 19:  smallHydroStop(); break;
      case 20:  smallHydroDown(); break;
      
      default:
        Serial.println("[ERROR] Unknown command!");
        break;
    }
  }
}

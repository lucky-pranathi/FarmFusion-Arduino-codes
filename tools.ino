#include "BluetoothSerial.h"

#define THROTTLE_PIN1 22
#define REVERSE_PIN   32   // Relay control for BLDC reverse wires
volatile int revFlag=0;

BluetoothSerial SerialBT;

/* ===== Hydraulic ===== */
#define RPWM 25
#define LPWM 33

#define rpwm_rotate 14
#define lpwm_rotate 17

#define sprayRPWM 27
#define sprayLPWM 26

/* ===== SPRAY VARIABLES ===== */
bool sprayEnabled = false;          
int spraySpeedPWM = 60;             

/* ===== ROTATION ===== */
int currentAngle = 0;
bool rotatingRight = false;
bool rotatingLeft = false;
int rotateSpeed = 150;

const int enablePin =2;

/* ===== HUB SPEED FROM SLIDER ===== */
int hubSpeedPWM = 100;   // default PWM for hub motor


/* ============================================================
      ROTATION FUNCTIONS
   ============================================================ */
void rotate90() {
  rotatingRight = true;
  rotatingLeft = false;

  analogWrite(rpwm_rotate, rotateSpeed);
  analogWrite(lpwm_rotate, 0);

  Serial.println(">> ROTATING RIGHT...");
  SerialBT.println("Rotating Right");
}

void rotate0() {
  rotatingLeft = true;
  rotatingRight = false;

  analogWrite(rpwm_rotate, 0);
  analogWrite(lpwm_rotate, rotateSpeed);

  Serial.println(">> ROTATING LEFT...");
  SerialBT.println("Rotating Left");
}

void stopRotate() {
  rotatingLeft = false;
  rotatingRight = false;

  analogWrite(rpwm_rotate, 0);
  analogWrite(lpwm_rotate, 0);

  Serial.println(">> ROTATION STOPPED");
  SerialBT.println("Rotation stopped");
}


/* ============================================================
        SPRAY MOTOR CONTROL
   ============================================================ */
void setMotorSpeed(int speedPercent) {
  spraySpeedPWM = map(speedPercent, 0, 100, 0, 255);

  Serial.print("Spray Speed Set: ");
  Serial.println(speedPercent);

  if (sprayEnabled) {
    analogWrite(sprayRPWM, spraySpeedPWM);
    analogWrite(sprayLPWM, 0);
  } else {
    analogWrite(sprayRPWM, 0);
    analogWrite(sprayLPWM, 0);
  }
}


/* ============================================================
        HUB SPEED CONTROL (NEW)
   ============================================================ */
void setHubSpeed(int speedPercent) {
  hubSpeedPWM = map(speedPercent, 0, 100, 0, 255);

  Serial.print("Hub Speed Set: ");
  Serial.println(speedPercent);
  SerialBT.println("Hub Speed: " + String(speedPercent));
}


/* ============================================================
        VEHICLE (HUB MOTOR) CONTROL
   ============================================================ */
void startHub(){
  if(revFlag==1){
    digitalWrite(REVERSE_PIN, HIGH); 
    delay(200);
    digitalWrite(REVERSE_PIN,LOW);
    revFlag=0;
  }

  analogWrite(THROTTLE_PIN1, hubSpeedPWM);   // UPDATED
  Serial.println("Motors ON");
  SerialBT.println("Motors running");
}

void stopHub(){
  analogWrite(THROTTLE_PIN1, 0);
  Serial.println("Motors OFF");
  SerialBT.println("Motors stopped");
}

void reverse(){
  if (revFlag==0){
    digitalWrite(REVERSE_PIN, HIGH);
    delay(200);
    digitalWrite(REVERSE_PIN,LOW);
    revFlag=1;
  }

  analogWrite(THROTTLE_PIN1, hubSpeedPWM);  // UPDATED
  Serial.println("Moving Backward");
  SerialBT.println("Moving Backward");
}


/* ============================================================
                  HYDRAULIC CONTROL
   ============================================================ */
void hydroDown() {
  Serial.println(">> HYDRAULIC DOWN");
  SerialBT.println("Hydraulic moving DOWN");
  analogWrite(RPWM, 180);
  analogWrite(LPWM, 0);
}

void hydroUp() {
  Serial.println(">> HYDRAULIC UP");
  SerialBT.println("Hydraulic moving UP");
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 180);
}

void hydroStop() {
  Serial.println(">> HYDRAULIC STOP");
  SerialBT.println("Hydraulic STOPPED");
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}


/* ============================================================
                    SPRAY ACTUATOR
   ============================================================ */
void activateSpray(){
  analogWrite(sprayRPWM, 180);
  analogWrite(sprayLPWM, 0);
}

void deactivateSpray(){
  analogWrite(sprayRPWM, 0);
  analogWrite(sprayLPWM, 180);
}

void stopSpray(){
  analogWrite(sprayRPWM, 0);
  analogWrite(sprayLPWM, 0);
}

void spray_on(){
  sprayEnabled = true;
  Serial.println("Spray ENABLED");
  SerialBT.println("Spray ENABLED");
  analogWrite(sprayRPWM, spraySpeedPWM);
  analogWrite(sprayLPWM, 0);
}

void spray_off(){
  sprayEnabled = false;
  Serial.println("Spray DISABLED");
  SerialBT.println("Spray DISABLED");
  analogWrite(sprayRPWM, 0);
  analogWrite(sprayLPWM, 0);
}


/* ============================================================
                        SETUP
   ============================================================ */
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_MOTOR");

  pinMode(THROTTLE_PIN1, OUTPUT);
  analogWrite(THROTTLE_PIN1,0);
  pinMode(REVERSE_PIN, OUTPUT);
  digitalWrite(REVERSE_PIN,LOW);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(rpwm_rotate,OUTPUT);
  pinMode(lpwm_rotate,OUTPUT);
  pinMode(sprayRPWM,OUTPUT);
  pinMode(sprayLPWM,OUTPUT);
  
  pinMode(enablePin,OUTPUT);
  digitalWrite(enablePin,HIGH);
}


/* ============================================================
                        MAIN LOOP
   ============================================================ */
void loop() {
  if (SerialBT.available()) {

    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    int code = -1;

    if (cmd == "FORWARD")        code = 1;
    else if (cmd == "STOP")      code = 2;
    else if (cmd == "BACKWARD")  code = 3;
    else if (cmd.startsWith("SPD:")) code = 4;
    else if (cmd.startsWith("HSPD:")) code = 18;  // NEW → hub slider
    else if (cmd == "ROTATE_90") code = 5;
    else if (cmd == "ROTATE_0")  code = 6;
    else if(cmd=="stop_rotate") code=17;
    else if (cmd=="MOVE_DOWN_HYDRO") code=7;
    else if(cmd=="MOVE_UP_HYDRO") code=8;
    else if(cmd=="STOP_HYDRO") code=9;
    else if(cmd=="HARVEST_ON") code=10;
    else if(cmd=="HARVEST_OFF") code=11;
    else if(cmd== "spray_on") code=12;
    else if(cmd=="spray_off") code=13;
    else if(cmd=="activate_spray") code=14;
    else if(cmd=="deactivate_spray") code=15;
    else if(cmd=="stop_actuator") code=16;


    /* ===== COMMAND HANDLING ===== */
    switch (code) {

      case 1:
        startHub();
        break;

      case 2:
        stopHub();
        break;

      case 3:
        reverse();
        break;

      case 4: {
        String val = cmd.substring(4);
        int speedPercent = val.toInt();
        if (speedPercent >= 0 && speedPercent <= 100)
          setMotorSpeed(speedPercent);
        break;
      }

      case 18: {   // NEW HUB SPEED
        String val = cmd.substring(5);
        int speedPercent = val.toInt();
        if (speedPercent >= 0 && speedPercent <= 100)
          setHubSpeed(speedPercent);
        break;
      }

      case 5:
        rotate90();
        break;

      case 6:
        rotate0();
        break;

      case 17:
        stopRotate();
        break;

      case 7:
        hydroDown();
        break;

      case 8:
        hydroUp();
        break;

      case 9:
        hydroStop();
        break;

      case 12:
        spray_on();
        break;

      case 13:
        spray_off();
        break;

      case 14:
        activateSpray();
        break;

      case 15:
        deactivateSpray();
        break;

      case 16:
        stopSpray();
        break;

      default:
        Serial.println("Unknown Command");
        break;
    }
  }
}

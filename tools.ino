#include "BluetoothSerial.h"

#define THROTTLE_PIN1 26
#define REVERSE_PIN   32   // Relay control for BLDC reverse wires
volatile int revFlag=0;

BluetoothSerial SerialBT;

/* ===== Hydraulic ===== */
#define RPWM 25
#define LPWM 33

#define rpwm_rotate 14
#define lpwm_rotate 17

#define sprayRPWM 35
#define sprayLPWM 34

/* ===== ADD THESE VARIABLES ===== */
bool sprayEnabled = false;         // spray_on() must turn this ON
int spraySpeedPWM = 0;             // stores PWM after SPD:X


// int speedVal = 150;              // rotation motor speed
// int fullRotationTime = 2000;     // rotation time for 360 degrees
// int angleVal = 90;               // required rotation angle → 90 degrees

int currentAngle = 0;          // store current angle (0 to 90)
bool rotatingRight = false;
bool rotatingLeft = false;
int rotateSpeed = 100;          // slow, safe rotation PWM

const int enablePin =2;


/* ============================================================
      ROTATION FUNCTIONS ADDED (NO OTHER CHANGES)
   ============================================================ */

void rotate90() {
  if (currentAngle >= 90) {
    Serial.println(">> Already at MAX 90°, cannot rotate further RIGHT");
    SerialBT.println("Max angle reached");
    return;
  }

  rotatingRight = true;
  rotatingLeft = false;

  analogWrite(rpwm_rotate, rotateSpeed);
  analogWrite(lpwm_rotate, 0);

  Serial.println(">> ROTATING RIGHT...");
  SerialBT.println("Rotating Right");
}

void rotate0() {
  if (currentAngle <= 0) {
    Serial.println(">> Already at MIN 0°, cannot rotate further LEFT");
    SerialBT.println("Min angle reached");
    return;
  }

  rotatingLeft = true;
  rotatingRight = false;

  analogWrite(rpwm_rotate, 0);
  analogWrite(lpwm_rotate, rotateSpeed);

  Serial.println(">> ROTATING LEFT...");
  SerialBT.println("Rotating Left");
}

void stopRotate(){
  rotatingLeft = false;
  rotatingRight = false;

  analogWrite(rpwm_rotate, 0);
  analogWrite(lpwm_rotate, 0);

  Serial.println(">> ROTATION STOPPED");
  SerialBT.println("Rotation stopped");
}

/* ===== Update current angle in loop while rotating ===== */
void updateRotationAngle() {

  if (rotatingRight) {
    if (currentAngle < 90) {
      currentAngle++;
    } else {
      stopRotate();
    }
    delay(20);  // slow update = smooth movement
  }

  if (rotatingLeft) {
    if (currentAngle > 0) {
      currentAngle--;
    } else {
      stopRotate();
    }
    delay(20);
  }
}

void setMotorSpeed(int speedPercent) {
  spraySpeedPWM = map(speedPercent, 0, 100, 0, 255);

  Serial.print("Spray Speed Set: ");
  Serial.println(speedPercent);

  // Apply speed ONLY if spray is enabled
  if (sprayEnabled) {
    analogWrite(sprayRPWM, spraySpeedPWM);
    analogWrite(sprayLPWM, 0);
  } else {
    // spray is OFF → keep spray motor stopped
    analogWrite(sprayRPWM, 0);
    analogWrite(sprayLPWM, 0);
  }
}

void startHub(){
  int speed = 100;
  if(revFlag==1){
    digitalWrite(REVERSE_PIN, HIGH); 
    delay(200);
    digitalWrite(REVERSE_PIN,LOW);
    revFlag=0;
  }
  analogWrite(THROTTLE_PIN1, speed);
  Serial.println("Motors ON at speed 90");
  SerialBT.println("Motors running at 90");
}

void stopHub(){
  analogWrite(THROTTLE_PIN1, 0);
  Serial.println("Motors OFF");
  SerialBT.println("Motors stopped");
}

void reverse(){
  digitalWrite(REVERSE_PIN, HIGH);
  delay(200);
  digitalWrite(REVERSE_PIN,LOW);
  revFlag=1;
  int speed = 100;
  analogWrite(THROTTLE_PIN1, speed);
  Serial.println("Moving Backward");
  SerialBT.println("Moving Backward");
}

/* ============================================================
                  HYDRAULIC LIFT CONTROL FUNCTIONS
   ============================================================ */

// MOVE HYDRAULIC DOWN (extend actuator)
void hydroDown() {
  Serial.println(">> HYDRAULIC DOWN");
  SerialBT.println("Hydraulic moving DOWN");

  // Rotate forward
  analogWrite(RPWM, 180);   // adjust speed if needed
  analogWrite(LPWM, 0);
}

// MOVE HYDRAULIC UP (retract actuator)
void hydroUp() {
  Serial.println(">> HYDRAULIC UP");
  SerialBT.println("Hydraulic moving UP");

  // Rotate backward
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 180);   // adjust speed if needed
}

// STOP HYDRAULIC ACTUATOR
void hydroStop() {
  Serial.println(">> HYDRAULIC STOP");
  SerialBT.println("Hydraulic STOPPED");

  // Stop movement
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

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

  // If a speed was already received earlier, apply it now
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

/* ===== ORIGINAL SETUP (unchanged) ===== */
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_MOTOR");
  Serial.println("Bluetooth device ready. Pair with 'ESP32_MOTOR'");

  pinMode(THROTTLE_PIN1, OUTPUT);
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

void loop() {
  updateRotationAngle();

  if (SerialBT.available()) {

    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    int code = -1;

    /* ===== COMMAND PARSING ===== */
    if (cmd == "FORWARD")        code = 1;    // Move vehicle forward
    else if (cmd == "STOP")      code = 2;    // Stop vehicle
    else if (cmd == "BACKWARD")  code = 3;    // Move backward
    else if (cmd.startsWith("SPD:")) code = 4; // Set spray motor speed
    else if (cmd == "ROTATE_90") code = 5;    // Rotate implement 90 degrees
    else if (cmd == "ROTATE_0")  code = 6;      //rotate 0
    else if(cmd=="stop_rotate") code=17;    // Rotate implement back to 0 degrees
    else if (cmd=="MOVE_DOWN_HYDRO") code=7;  // Hydraulic down
    else if(cmd=="MOVE_UP_HYDRO") code=8;    // Hydraulic up
    else if(cmd=="STOP_HYDRO") code=9;       // Stop hydraulic
    else if(cmd=="HARVEST_ON") code=10;      // Start harvester
    else if(cmd=="HARVEST_OFF") code=11;     // Stop harvester
    else if(cmd== "spray_on") code=12;      //spray on
    else if(cmd=="spray_off") code=13;      //spray off
    else if (cmd=="activate_spray") code=14;         //activate spray
    else if(cmd=="deactivate_spray") code=15;      //deactivate spray
    else if(cmd=="stop_actuator") code=16;         //stop spray


    /* ===== COMMAND HANDLING ===== */
    switch (code) {

      case 1:   // FORWARD
        startHub();
        break;

      case 2:   // STOP
        stopHub();
        break;

      case 3:   // BACKWARD
        reverse();
        break;

      case 4: { // SPD:X → spray speed
        String val = cmd.substring(4);
        int speedPercent = val.toInt();
        if (speedPercent >= 0 && speedPercent <= 100) {
          setMotorSpeed(speedPercent);
        }
        break;
      }

      case 5:   // ROTATE_90
        rotate90();
        break;

      case 6:   // ROTATE_0
        rotate0();
        break;

      case 7:   // MOVE_DOWN_HYDRO
        hydroDown();
        break;

      case 8:   // MOVE_UP_HYDRO
        hydroUp();
        break;

      case 9:   // STOP_HYDRO
        hydroStop();
        break;

      case 10:  // HARVEST_ON
        break;

      case 11:  // HARVEST_OFF
        break;

      case 12:    //spray on
        spray_on();
        break;

      case 13:      //spray off
        spray_off();
        break;

      case 14:     //activate spray
        activateSpray();
        break;

      case 15:       //deactivate spray
        deactivateSpray();
        break;

      case 16:          //stop spray
        stopSpray();
        break;

      case 17:         //stop rotation
        stopRotate();
        break;


      default:
        Serial.println("Unknown Command");
        break;
    }
  }
}

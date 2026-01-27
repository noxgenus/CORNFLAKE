/*
 * ----------------------------------------------------------------------------
   SUPERHEROES R2D2
   ----------------------------------------------------------------------------
   Teensy 4.0 / L298N based Robot Controller
   2026
   ----------------------------------------------------------------------------

*/
String MBversion = "1.0";
/*

   ----------------------------------------------------------------------------
   TEENSY 4.0 Pin layout:

   0 - RX1                    REMOTE TX
   1 - TX1
   
   2 - Motor LEFT PWM         (PWM) /EN1
   3 - Motor LEFT 1           (PWM) /1
   4 - Motor LEFT 2           (PWM) /2
   5 - Motor RIGHT PWM        (PWM) /EN2
   6 - Motor RIGHT 1          (PWM) /3
   7 - Motor RIGHT 2          (PWM) /4
   8 - Motor HEAD PWM         (PWM) /EN3
   9 - Motor HEAD 1           (PWM) /5
   10 - Motor HEAD 2          (PWM) /6
   11 - EMPTY
   12 - EMPTY

   13 - EMPTY
   14 - EMPTY
   15 - EMPTY
   16 - RX4                   
   17 - TX4
   18 - OLED SDA              
   19 - OLED SCL
   20 - EMPTY                 (NON-PWM)
   21 - EMPTY                 (NON PWM)
   22 - EMPTY                 (PWM)
   23 - EMPTY                 (PWM)
  
  t5
   

   WARNING!
   1- If sketch increases, tune delay of remote! (if wireless!)

*/


#include <SPI.h>
#include <Servo.h>

int pulseLed1 = 4000;
int pulseLed2 = 1000;
int pulseIT = 10;
int pulseIT2 = 10;
bool runSeq = false;
bool busyLed1 = false;
bool busyLed2 = false;


// ==================================================================================
// ----------------------------------  SERVOS  --------------------------------------
// ==================================================================================

// CHARS FOR TFT TEXT OUTPUT STRINGS
char M1char[4];
char M2char[4];
char servo1char[4];
char servo2char[4];
char servo3char[4];
char servo4char[4];
char servo5char[4];
char servo6char[4];
char servo7char[4];
char servo8char[4];


int posipan = 90;
int positilt = 90;

Servo servoHead;

// Common servo setup values
int minPulseHITEC = 900;    // minimum servo position, us (microseconds)
int maxPulseHITEC = 2100;    // maximum servo position, us

int minPulseFUTUBA = 1000;    // minimum servo position, us (microseconds)
int maxPulseFUTUBA = 2000;    // maximum servo position, us


// HC12 SERIAL INPUT SERVO/VALUE
int userInput[3];    // raw input from serial buffer, 3 bytes
int startbyte;       // start byte, begin reading input
int servo;           // which servo to pulse?
int pos;             // servo angle 0-180
int i;               // iterator

// Serial timeout on lost connection
const long timeout = 1000;
long counter = 0;

// OLED DISPLAY INFO LOOP
unsigned long lastStartStopTime = 0;
int OLEDinterval = 200; // millisec


// ==============================================================================
// -------------------------- VALUES ARRAY --------------------------------------
// ==============================================================================


// Keep init servo positions in boot and general int array (we're adding and subtracting from these init values with encoder!)
int lastKnownPos[17] = {0, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};

String valM1 = String(lastKnownPos[1]);
String valM2 = String(lastKnownPos[2]);
String valServo1 = String(lastKnownPos[3]);
String valServo2 = String(lastKnownPos[4]);
String valServo3 = String(lastKnownPos[5]);
String valServo4 = String(lastKnownPos[6]);
String valServo5 = String(lastKnownPos[7]);
String valServo6 = String(lastKnownPos[8]);
String valServo7 = String(lastKnownPos[9]);
String valServo8 = String(lastKnownPos[10]);
String valServo9 = String(lastKnownPos[11]);
String valServo10 = String(lastKnownPos[12]);

String valButt1 = String(lastKnownPos[13]);
String valButt2 = String(lastKnownPos[14]);
String valButt3 = String(lastKnownPos[15]);
String valButt4 = String(lastKnownPos[16]);


// TIME
unsigned long now2 = millis();  // NOW2 FOR DEBOUNCE

// ==============================================================================
// ---------------------------- i2C OLED ----------------------------------------
// ==============================================================================

#include <Wire.h>

// Wire1.setSCL(19);
// Wire1.setSDA(18);

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#define i2c_Address 0x3c
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


// ===================== LEDS ========================= //

// HEAD LEDS
const int led1 = 2;
const int led2 = 3;
const int led3 = 4;

// ===================== MOTORS ========================= //

// LEG MOTOR LEFT
const int IN1 = 6;
const int IN2 = 7;
const int ENA = 5; // Needs to be a PWM pin to be able to control motor speed

// LEG MOTOR RIGHT
const int IN3 = 9;
const int IN4 = 10;
const int ENB = 8; // Needs to be a PWM pin to be able to control motor speed


// ================== MOTOR TUNING ================ //

const int   JOY_CENTER = 90;
const int   JOY_MIN = 0;
const int   JOY_MAX = 180;
const int   DEADZONE = 2;        // around 90 (try 2–8)
const uint8_t MIN_START_PWM = 0; // e.g. 40 if motors stall

int joyX = 90; // 0..180
int joyY = 90; // 0..180


// ======================== DEBUG ====================== //

// Debug on/off
boolean debug = false;
boolean busyflag = false;


void setup() {

  Serial.begin(9600);   // debug monitor
  Serial1.begin(9600);  // HC12 C044 (RX) (CONTROL INPUT)

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);


  servoHead.attach(11, minPulseHITEC, maxPulseHITEC);

  display.begin(0x3C, true); // Address 0x3C default
  display.setRotation(0); 
  delay(250); // wait for the OLED to power up
  // Clear the buffer.
  display.clearDisplay();
  oledAll();


  driveMixedTank(0, 0);

    delay(1000);
    Serial.println("R2D2 V1");
    Serial.println("READY!");

}

/*
      move(1, val1);    //fwd/rev joy
      move(2, val2);    //steering joy
      move(3, val3);    //? top joy rotate
      move(4, val4);    //tilt joypad
      move(5, val5);    //pan joypad
      move(6, val6);    //shoulder L pot
      move(7, val7);    // arm L pot
      move(8, val8);    //shoulder R pot
      move(9, val9);    //arm R pot
      move(10, val10);  //gripper L
      move(11, val11);  //gripper R
      move(12, val12);  //rc led contrller

      move(20, val13);  //button blue
      move(21, val14);  //button yellow
      move(22, val15);  //button red
      move(23, val16);  //button joystick
*/

void loop() {

  now2 = millis();

  // ========================= LED PULSING =====================================

  pulseIT = 128+107*cos(2*PI/pulseLed1*now2);
  pulseIT2 = 128+127*cos(2*PI/pulseLed2*now2+255);

  analogWrite(led1,pulseIT);
  // analogWrite(led2,pulseIT2);
  analogWrite(led3,0);

  // ====================== OLED INTERVAL ======================================

  if ((lastStartStopTime + (OLEDinterval)) < now2) {
    oledAll();
    lastStartStopTime = now2;
  }


  // =================== NO SERIAL CONNECTION ==================================

  counter = counter + 1;
  if (counter >= timeout) {
    
    if (debug == true) {Serial.println("No connection!");}

    // SET motors and servos into neutral position and update array
    lastKnownPos[1] = 90;
    lastKnownPos[2] = 90;
    lastKnownPos[3] = 90;
    lastKnownPos[4] = 90;
    lastKnownPos[5] = 90;
    lastKnownPos[6] = 90;
    lastKnownPos[7] = 90;
    lastKnownPos[8] = 90;
    lastKnownPos[9] = 90;
    lastKnownPos[10] = 90;
    lastKnownPos[11] = 90;
    lastKnownPos[12] = 90;

    analogWrite(led3,0);

    joyX = 90; // 0..180
    joyY = 90; // 0..180

  } 


//=========================================================================
// -----------------------| SERIAL RADIO CONTROL |-------------------------
//=========================================================================

  if (Serial1.available() > 2) {

   counter = 0; // Reset connection timeout counter
   analogWrite(led3,255);

   startbyte = Serial1.read();

    if (startbyte == 255) {
      for (i = 0; i < 2; i++) {
        userInput[i] = Serial1.read();
      }
     servo = userInput[0];
     pos = userInput[1];
     if (pos == 255) {
       servo = 255;
     }

      switch (servo) {
        case 1: // DRIVE
          if (debug == true) {Serial.print("fwd/rev: ");Serial.print(pos);}
          //pos = map(pos, 0, 180, 180, 0); // FWD
          lastKnownPos[1] = pos;
          joyY = lastKnownPos[1];
          break;
        case 2: // STEERING
          if (debug == true) {Serial.print(" steer: ");Serial.print(pos);}
          pos = map(pos, 0, 180, 180, 0); // REVERSE
          lastKnownPos[2] = pos;
          joyX = lastKnownPos[2];
          break;
        case 3: // top joy rotate
          if (debug == true) {Serial.print(" head rotate: ");Serial.print(pos);}
       
            if (pos > 95) {
                posipan = (posipan + 2); // slow
          
              } else if (pos < 85) {
                posipan = (posipan - 2); // faster
              }
              if (posipan > 180) posipan = 180; //limit upper value
              if (posipan < 0) posipan = 0; //limit lower value

            lastKnownPos[3] = posipan;
            servoHead.write(lastKnownPos[3]);
          break;
        case 4: // TILT
          if (debug == true) {Serial.print(" tilt: ");Serial.print(pos);}
            lastKnownPos[4] = pos;
          break;
        case 5: // PAN
          if (debug == true) {Serial.print(" pan: ");Serial.print(pos);}
            lastKnownPos[5] = pos;
          break;
        case 6: // SHOULDER L
          if (debug == true) {Serial.print(" shoulderL: ");Serial.print(pos);}
            lastKnownPos[6] = pos;
          break;
        case 7: // ARM L
          if (debug == true) {Serial.print(" armL: ");Serial.print(pos);}
            lastKnownPos[7] = pos;
          break;
        case 8: // SHOULDER R
          if (debug == true) {Serial.print(" shoulderR: ");Serial.print(pos);}
            lastKnownPos[8] = pos;
          break;
        case 9: // ARM R
          if (debug == true) {Serial.print(" armR: ");Serial.print(pos);}
            lastKnownPos[9] = pos;
          break;
        case 10: // GRIPPER L
          if (debug == true) {Serial.print(" gripperL: ");Serial.println(pos);}
            lastKnownPos[10] = pos;
          break;
        case 11: // GRIPPER R
            lastKnownPos[11] = pos;
          break;
        case 12: // LED CONTROLLER
            lastKnownPos[12] = pos;
          break;
        case 20: // blue button
            lastKnownPos[13] = pos;
            if (pos == 0) {
                 analogWrite(led2,0);
              } else if (pos == 1) {
                  analogWrite(led2,pulseIT2);
            }
             
          break;
        case 21: // yellow button
            lastKnownPos[14] = pos;
          break;
        case 22: // red button
            lastKnownPos[15] = pos;
          break;
        case 23: // fire button
            lastKnownPos[16] = pos;
          break;
        

        }
      }
    
  }


  int16_t turn     = joy180ToCmd(joyX);
  int16_t throttle = joy180ToCmd(joyY);

  driveMixedTank(throttle, turn);

  delay(2);
}



void oledAll(){

    valM1 = String(lastKnownPos[1]);
    valM2 = String(lastKnownPos[2]);
    valServo1 = String(lastKnownPos[4]);
    valServo2 = String(lastKnownPos[5]);
    valServo3 = String(lastKnownPos[6]);
    valServo4 = String(lastKnownPos[7]);
    valServo5 = String(lastKnownPos[8]);
    valServo6 = String(lastKnownPos[9]);
    valServo7 = String(lastKnownPos[10]);
    valServo8 = String(lastKnownPos[11]);
    valServo9 = String(lastKnownPos[12]);
    valServo10 = String(lastKnownPos[13]);


  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.setCursor(0,0);
  display.println("MOT1:");
  display.setCursor(40,0);
  valM1.toCharArray(M1char, 4);
  display.println(M1char);

  display.setCursor(0,10);
  display.println("MOT2:");
  display.setCursor(40,10);
  valM2.toCharArray(M2char, 4);
  display.println(M2char);

  display.setCursor(0,20);
  display.println("TILT:");
  display.setCursor(40,20);
  valServo1.toCharArray(servo1char, 4);
  display.println(servo1char);

  display.setCursor(0,30);
  display.println("PAN:");
  display.setCursor(40,30);
  valServo2.toCharArray(servo2char, 4);
  display.println(servo2char);


  display.setCursor(0,40);
  display.println("SHO-L:");
  display.setCursor(40,40);
  valServo3.toCharArray(servo3char, 4);
  display.println(servo3char);


  display.setCursor(0,50);
  display.println("ARM-L:");
  display.setCursor(40,50);
  valServo4.toCharArray(servo4char, 4);
  display.println(servo4char);

  display.setCursor(70,0);
  display.println("SHO-R:");
  display.setCursor(110,0);
  valServo5.toCharArray(servo5char, 4);
  display.println(servo5char);

  display.setCursor(70,10);
  display.println("ARM-R:");
  display.setCursor(110,10);
  valServo6.toCharArray(servo6char, 4);
  display.println(servo6char);

  display.setCursor(70,20);
  display.println("GRIP:");
  display.setCursor(110,20);
  valServo7.toCharArray(servo7char, 4);
  display.println(servo7char);



  display.display();
}


// ---------- Helpers ----------
static inline int16_t clamp255(int16_t v) {
  if (v > 255) return 255;
  if (v < -255) return -255;
  return v;
}

static inline int applyDeadzone(int v, int dz) {
  if (v > -dz && v < dz) return 0;
  return v;
}

// Convert joystick angle-style value (0..180) into motor command (-255..255)
int16_t joy180ToCmd(int joyVal) {
  int delta = joyVal - JOY_CENTER;     // now -90 .. +90
  delta = applyDeadzone(delta, DEADZONE);

  if (delta > 90) delta = 90;
  if (delta < -90) delta = -90;

  long cmd = map(delta, -90, 90, -255, 255);
  return (int16_t)cmd;
}

// Set one motor
void setMotorL298N(uint8_t en, uint8_t inA, uint8_t inB, int16_t cmd) {
  cmd = clamp255(cmd);

  if (cmd == 0) {
    // COAST. For brake, set both HIGH.
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(en, 0);
    return;
  }

  bool forward = (cmd > 0);
  uint8_t pwm = abs(cmd);

  if (MIN_START_PWM > 0 && pwm > 0 && pwm < MIN_START_PWM)
    pwm = MIN_START_PWM;

  digitalWrite(inA, forward ? HIGH : LOW);
  digitalWrite(inB, forward ? LOW  : HIGH);
  analogWrite(en, pwm);
}

// Tank mix
void driveMixedTank(int16_t throttle, int16_t turn) {
  int16_t left  = clamp255(throttle + turn);
  int16_t right = clamp255(throttle - turn);

  setMotorL298N(ENA, IN1, IN2, left);
  setMotorL298N(ENB, IN3, IN4, right);
}
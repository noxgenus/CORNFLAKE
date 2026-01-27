/*
 * ----------------------------------------------------------------------------
   MAGIC BULLET AMSTERDAM 2024
   ----------------------------------------------------------------------------
   Teensy 4.0 based Robot Controller
   In combination with Nvidia Jetson Nano and GPT4
   Using custom 16bit servo library
   ----------------------------------------------------------------------------

*/
String MBversion = "1.0";
/*

   ----------------------------------------------------------------------------
   TEENSY 4.0 Pin layout:

   0 - RX1                    REMOTE TX
   1 - TX1
   
   2 - Motor Left             (PWM) /SABER IN1
   3 - Motor right            (PWM) /SABER IN2
   4 - Servo 1                (PWM) /PAN
   5 - Servo 2                (PWM) /TILT
   6 - Servo 3                (PWM) /SHOULDER L
   7 - Servo 4                (PWM) /ELBOW L
   8 - Servo 5                (PWM) /SHOULDER R
   9 - Servo 6                (PWM) /ELBOW R
   10 - Servo 7               (PWM) /GRIPPER
   11 - EMPTY
   12 - EMPTY

   13 - EMPTY
   14 - Blue led 1
   15 - Blue led 2
   16 - RX4                   NODE TX
   17 - TX4
   18 - OLED SDA              
   19 - OLED SCL
   20 - EMPTY                 (NON-PWM)
   21 - EMPTY                 (NON PWM)
   22 - NEOPIXELS             (PWM)
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

int blueFlashFront1 = 14;
int blueFlashFront2 = 15;


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


const int panOffset = 5;

int posipan = 90;
int positilt = 90;

Servo motor1; // FWD
Servo motor2; // TURN
Servo servo1; // TILT HEAD
Servo servo2; // PAN HEAD
Servo servo3; // SHOULDER L
Servo servo4; // ARM L
Servo servo5; // SHOULDER R
Servo servo6; // ARM R
Servo servo7; // GRIPPER
Servo servo8; // LED BLINKIE CONTROLLER

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
const long timeout = 200;
long counter = 0;

// OLED DISPLAY INFO LOOP
unsigned long lastStartStopTime = 0;
int OLEDinterval = 200; // millisec

// Debug on/off
boolean debug = false;
boolean busyflag = false;

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

void setup() {

  Serial.begin(9600);   // debug monitor
  Serial1.begin(9600);  // HC12 C044 (RX) (CONTROL INPUT)

  pinMode(blueFlashFront1, OUTPUT);
  pinMode(blueFlashFront2, OUTPUT);


   // Teensy 4.0 Servo pin map
    motor1.attach(2, minPulseFUTUBA, maxPulseFUTUBA);
    motor2.attach(3, minPulseFUTUBA, maxPulseFUTUBA);
    servo1.attach(4, minPulseHITEC, maxPulseHITEC);
    servo2.attach(5, minPulseHITEC, maxPulseHITEC);
    servo3.attach(6, minPulseHITEC, maxPulseHITEC);
    servo4.attach(7, minPulseHITEC, maxPulseHITEC);
    servo5.attach(8, minPulseHITEC, maxPulseHITEC);
    servo6.attach(9, minPulseFUTUBA, maxPulseFUTUBA);
    servo7.attach(10, minPulseFUTUBA, maxPulseFUTUBA);
    servo8.attach(23, minPulseFUTUBA, maxPulseFUTUBA);

    // Set servos to static array setting
    motor1.write(lastKnownPos[1]);
    motor2.write(lastKnownPos[2]);
    // UNUSED: lastKnownPos[3];
    servo1.write(lastKnownPos[4]);
    servo2.write(lastKnownPos[5]);
    servo3.write(lastKnownPos[6]);
    servo4.write(lastKnownPos[7]);
    servo5.write(lastKnownPos[8]);
    servo6.write(lastKnownPos[9]);
    servo7.write(lastKnownPos[10]);
    // UNUSED: servo??.write(lastKnownPos[11]);
    servo8.write(lastKnownPos[12]);


  display.begin(0x3C, true); // Address 0x3C default
  display.setRotation(2); 
  delay(250); // wait for the OLED to power up
  // Clear the buffer.
  display.clearDisplay();
  oledAll();

    // pixels.show();

    delay(1000);
    Serial.println("CORNFLAKE ROBOT CONTROL");
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

  pulseIT = 128+107*cos(2*PI/pulseLed1*now2);
  pulseIT2 = 128+127*cos(2*PI/pulseLed2*now2+255);

  analogWrite(blueFlashFront1,pulseIT2);
  analogWrite(blueFlashFront2,pulseIT2);

  if ((lastStartStopTime + (OLEDinterval)) < now2) {
    oledAll();
    lastStartStopTime = now2;
  }

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
    //lastKnownPos[11] = 90;
    lastKnownPos[12] = 90;

    // Set servos to static array setting
    motor1.write(lastKnownPos[1]);
    motor2.write(lastKnownPos[2]);
    // unused
    servo1.write(lastKnownPos[4]);
    servo2.write(lastKnownPos[5]);
    servo3.write(lastKnownPos[6]);
    servo4.write(lastKnownPos[7]);
    servo5.write(lastKnownPos[8]);
    servo6.write(lastKnownPos[9]);
    servo7.write(lastKnownPos[10]);
    //unused
    servo8.write(lastKnownPos[12]);


  } 


//====================================================================
// ---------------------| SERIAL RADIO CONTROL |----------------------
//====================================================================

  if (Serial1.available() > 2) {

   counter = 0; // Reset connection timeout counter
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
          Serial.print("fwd/rev: ");Serial.println(pos);
          motor1.write(pos);
          lastKnownPos[1] = pos;
          break;
        case 2: // STEERING
          if (debug == true) {Serial.print(" steer: ");Serial.print(pos);}
          //pos = map(pos, 0, 180, 180, 0); // REVERSE
          motor2.write(pos);
          lastKnownPos[2] = pos;
          break;
        case 3: // top joy rotate
          if (debug == true) {Serial.print(" top joy: ");Serial.print(pos);}
          //motor2.write(pos); ?? (NOT IN USE!!!)
          lastKnownPos[3] = pos;
          break;
        case 4: // TILT
          if (debug == true) {Serial.print(" tilt: ");Serial.print(pos);}
            if ((pos > 100) && (pos < 160)) {
              positilt = (positilt + 1); // slow
            } else if (pos > 160) {
              positilt = (positilt + 2); // faster
            } else if ((pos < 80) && (pos > 20)) {
              positilt = (positilt - 1); // slow
            } else if (pos < 20) {
              positilt = (positilt - 2); // faster
            }
            if (positilt > 180) positilt = 180; //limit upper value
            if (positilt < 0) positilt = 0; //limit lower value
            servo1.write(positilt);
            lastKnownPos[4] = positilt;
          break;
        case 5: // PAN
          if (debug == true) {Serial.print(" pan: ");Serial.print(pos);}
            if ((pos > 100) && (pos < 160)) {
              posipan = (posipan + 1); // slow
            } else if (pos > 160) {
              posipan = (posipan + 2); // faster
            } else if ((pos < 80) && (pos > 20)) {
              posipan = (posipan - 1); // slow
            } else if (pos < 20) {
              posipan = (posipan - 2); // faster
            }
            if (posipan > 180) posipan = 180; //limit upper value
            if (posipan < 0) posipan = 0; //limit lower value
            servo2.write(posipan);
            lastKnownPos[5] = posipan;
          break;
        case 6: // SHOULDER L
          if (debug == true) {Serial.print(" shoulderL: ");Serial.print(pos);}
          servo3.write(pos);
          lastKnownPos[6] = pos;
          break;
        case 7: // ARM L
          if (debug == true) {Serial.print(" armL: ");Serial.print(pos);}
          servo4.write(pos);
          lastKnownPos[7] = pos;
          break;
        case 8: // SHOULDER R
          if (debug == true) {Serial.print(" shoulderR: ");Serial.print(pos);}
          servo5.write(pos);
          lastKnownPos[8] = pos;
          break;
        case 9: // ARM R
          if (debug == true) {Serial.print(" armR: ");Serial.print(pos);}
          servo6.write(pos);
          lastKnownPos[9] = pos;
          break;
        case 10: // GRIPPER L
          if (debug == true) {Serial.print(" gripperL: ");Serial.println(pos);}
          servo7.write(pos);
          lastKnownPos[10] = pos;
          break;
        case 11: // GRIPPER R
          //servo7.write(pos);
          lastKnownPos[11] = pos;
          break;
        case 12: // LED CONTROLLER
          //servo8.write(pos);
          lastKnownPos[12] = pos;
          break;
        case 20: // blue button
          //servo8.write(pos);
          lastKnownPos[13] = pos;
          break;
        case 21: // yellow button
          //servo8.write(pos);
          lastKnownPos[14] = pos;
          break;
        case 22: // red button
          //servo8.write(pos);
          lastKnownPos[15] = pos;
          break;
        case 23: // fire button
          //servo8.write(pos);
          lastKnownPos[16] = pos;
          break;
        

        }
      }
    
  }



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

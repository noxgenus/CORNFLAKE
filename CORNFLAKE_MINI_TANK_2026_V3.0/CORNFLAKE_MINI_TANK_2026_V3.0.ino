/*
 * ----------------------------------------------------------------------------
   VW CREATIVE TECHNOLOGY
   ----------------------------------------------------------------------------
   Teensy 4.0 based Robot Controller
   With Arbotix AX12 Serial Controller
   ----------------------------------------------------------------------------

*/
String MBversion = "3.0";
/*

   ----------------------------------------------------------------------------
   TEENSY 4.0 Pin layout:

   0 - RX1                    (PWM) /HC 12 TX
   1 - TX1                    (PWM)
   2 - Motor Left             (PWM) /SABER IN1
   3 - Motor right            (PWM) /SABER IN2
   4 - Servo 1                (PWM) /PAN
   5 - Servo 2                (PWM) /TILT
   6 -                        (PWM) 
   7 -                        (PWM)
   8 -                        (PWM) 
   9 -                        (PWM)
   10 -                       (PWM)

   11 -                       (PWM)
   12 -                       (PWM)
   13 -                       (PWM)
   14 -                       (PWM)
   15 -                       (PWM)
   16 - RX4                   (PWM) /ARBOTIX TX
   17 - TX4                   (PWM) /ARBOTIX RX
   18 - OLED SDA              (PWM)
   19 - OLED SCL              (PWM)
   20 - TX5                   (PWM) /RX AUDIO
   21 - RX5                   (PWM) /TX AUDIO
   22 - APA102 DATA           (PWM)
   23 - APA102 CLK            (PWM)
  
  
   WARNING!
   1- If sketch increases, tune delay of remote! (if wireless!)

*/


#include <Servo.h>

const int panOffset = 5;

int posipan = 90;
int positilt = 90;

unsigned long lastStartStopTime = 0;
int Arbotixinterval = 50; // millisec

Servo motor1; // FWD
Servo motor2; // TURN
Servo servo1; // TILT HEAD
Servo servo2; // PAN HEAD

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

// Debug on/off
boolean debug = false;
boolean busyflag = false;

boolean leftArmActive = true;

// Keep init servo positions in boot and general int array (we're adding and subtracting from these init values with encoder!)
int lastKnownPos[20] = {0, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 0, 0, 0, 0};

/*
      move(1, val1);    //fwd/rev joy
      move(2, val2);    //steering joy
      move(3, val3);    //? top joy rotate
      move(4, val4);    //tilt joypad
      move(5, val5);    //pan joypad

      move(6, val6);    // AX12 ID1
      move(7, val7);    // AX12 ID2
      move(8, val8);    // AX12 ID3
      move(9, val9);    // AX12 ID4
      move(10, val10);  // AX12 ID5

      move(11, val11);  // AX12 ID6
      move(12, val12);  // AX12 ID7
      move(13, val13);  // AX12 ID8
      move(14, val14);  // AX12 ID9
      move(15, val15);  // AX12 ID10

      move(20, val16);  //button blue
      move(21, val17);  //button yellow
      move(22, val18);  //button red
      move(23, val19);  //button joystick
*/

// TIME
unsigned long now2 = millis();  // NOW2 FOR DEBOUNCE
#define DEBOUNCE_INTERVAL 500L // Milliseconds
unsigned long lastStartStopTime1 = 0;



void setup() {

  Serial.begin(9600);   // debug monitor
  Serial1.begin(9600);  // HC12 C044 (RX) (CONTROL INPUT)
  Serial4.begin(9600);  // ARBOTIX

   // Teensy 4.0 Servo pin map
    motor1.attach(3, minPulseFUTUBA, maxPulseFUTUBA);
    motor2.attach(4, minPulseFUTUBA, maxPulseFUTUBA);
    servo1.attach(6, minPulseHITEC, maxPulseHITEC);
    servo2.attach(7, minPulseHITEC, maxPulseHITEC);
   

    // Set servos to static array setting
    motor1.write(lastKnownPos[1]);
    motor2.write(lastKnownPos[2]);

    servo1.write(lastKnownPos[4]);
    servo2.write(lastKnownPos[5]);



   delay(1000);

    Serial.println("CORNFLAKE ROBOT CONTROLLER 2026");
    Serial.println("READY!");

}

void loop() {

  now2 = millis();



  counter = counter + 1;
  if (counter >= timeout) {
    
    if (debug == true) {Serial.println("No connection!");}

    // SET motors and servos into neutral position and update array
    lastKnownPos[1] = 90;
    lastKnownPos[2] = 90;

    lastKnownPos[4] = 90;
    lastKnownPos[5] = 90;
   
    // Set servos to static array setting
    motor1.write(lastKnownPos[1]);
    motor2.write(lastKnownPos[2]);
    // unused
    servo1.write(lastKnownPos[4]);
    servo2.write(lastKnownPos[5]);
    

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
          if (debug == true) {Serial.print("fwd/rev: ");Serial.println(pos);}
          pos = map(pos, 0, 180, 180, 0); // FWD
          motor1.write(pos);
          lastKnownPos[1] = pos;
          break;
        case 2: // STEERING
          if (debug == true) {Serial.print("steer: ");Serial.println(pos);}
          pos = map(pos, 0, 180, 180, 0); // REVERSE
          motor2.write(pos);
          lastKnownPos[2] = pos;
          break;
        case 3: // top joy rotate
          if (debug == true) {Serial.print("top joy: ");Serial.println(pos);}
          //motor2.write(pos); ?? (NOT IN USE!!!)
          //lastKnownPos[3] = pos;
          break;
        case 4: // TILT
          if (debug == true) {Serial.print("tilt: ");Serial.println(pos);}
            if (pos > 95) {
              positilt = (positilt + 5); // slow
            } else if (pos < 85) {
              positilt = (positilt - 5); // faster
            }
            if (positilt > 180) positilt = 180; //limit upper value
            if (positilt < 0) positilt = 0; //limit lower value
            servo1.write(positilt);
            lastKnownPos[4] = positilt;
          break;
        case 5: // PAN
          if (debug == true) {Serial.print("pan: ");Serial.println(pos);}
            if (pos > 95) {
              posipan = (posipan + 5); // faster
            } else if (pos < 85) {
              posipan = (posipan - 5); // faster
            }
            if (posipan > 180) posipan = 180; //limit upper value
            if (posipan < 0) posipan = 0; //limit lower value
            servo2.write(posipan);
            lastKnownPos[5] = posipan;
          break;
// LEFT ARM
        case 6: //BIOLOD SERVO ID1
          if (debug == true) {Serial.print("shoulderL: ");Serial.println(pos);}
          //pos = map(pos, 0, 180, 400, 600);
          if (leftArmActive == true){
       //     move(1, pos);  
            lastKnownPos[6] = pos;
          } else if (leftArmActive == false){
        //    move(6, pos);  
            lastKnownPos[11] = pos;
          }
          
          break;
        case 7: //BIOLOD SERVO ID2
          if (debug == true) {Serial.print("armL: ");Serial.println(pos);}
          //pos = map(pos, 0, 180, 400, 600);
          if (leftArmActive == true){
        //    move(2, pos);  
            lastKnownPos[7] = pos;
          } else if (leftArmActive == false){
         //   move(7, pos);  
            lastKnownPos[12] = pos;
          }
          
          break;
        case 8: //BIOLOD SERVO ID3
          if (debug == true) {Serial.print("shoulderR: ");Serial.println(pos);}
          //pos = map(pos, 0, 180, 400, 600);
          if (leftArmActive == true){
         //   move(3, pos);  
            lastKnownPos[8] = pos;
          } else if (leftArmActive == false){
          //  move(8, pos);  
            lastKnownPos[13] = pos;
          }
          
          break;
        case 9: //BIOLOD SERVO ID4
          if (debug == true) {Serial.print("armR: ");Serial.println(pos);}
         // pos = map(pos, 0, 180, 400, 600);
          if (leftArmActive == true){
            //move(4, pos);  
            lastKnownPos[9] = pos;
          } else if (leftArmActive == false){
           // move(9, pos);  
            lastKnownPos[14] = pos;
          } 
          break;
        case 10: //BIOLOD SERVO ID5
          if (debug == true) {Serial.print("gripperL: ");Serial.println(pos);}
         // pos = map(pos, 0, 180, 400, 600);
          if (leftArmActive == true){
            //move(5, pos); 
            lastKnownPos[10] = pos; 
          } else if (leftArmActive == false){
           // move(10, pos);  
            lastKnownPos[15] = pos;
          } 
          break;

// BUTTONS

        case 20: // switch (SET ARM L/R)
            if (pos == 0) {
                 leftArmActive = true;
              } else if (pos == 1) {
                leftArmActive = false;
            }
            lastKnownPos[16] = pos;
          break;
        case 21: // yellow button
            lastKnownPos[17] = pos;
          break;
        case 22: // red button
            lastKnownPos[18] = pos;
          break;
        case 23: // fire button
            lastKnownPos[19] = pos;
          break;
        

        }
      }
    
  }


 if ((lastStartStopTime + (Arbotixinterval)) < now2) {
    
      move(1,  lastKnownPos[6]);  
      move(6,  lastKnownPos[11]);  
      move(2, lastKnownPos[7]);  
      move(7, lastKnownPos[12]);  
      move(3,  lastKnownPos[8]);  
      move(8,  lastKnownPos[13]);  
      move(4, lastKnownPos[9]);  
      move(9, lastKnownPos[14]);  
      move(5, lastKnownPos[10]); 
      move(10, lastKnownPos[15]);  

    


    Serial.print("LEFT:");
    Serial.print(lastKnownPos[6]);
    Serial.print(':');
    Serial.print(lastKnownPos[7]);
    Serial.print(':');
    Serial.print(lastKnownPos[8]);
    Serial.print(':');
    Serial.print(lastKnownPos[9]);
    Serial.print(':');
    Serial.print(lastKnownPos[10]);
    
    Serial.print(" RIGHT:");
    Serial.print(lastKnownPos[11]);
    Serial.print(':');
    Serial.print(lastKnownPos[12]);
    Serial.print(':');
    Serial.print(lastKnownPos[13]);
    Serial.print(':');
    Serial.print(lastKnownPos[14]);
    Serial.print(':');
    Serial.println(lastKnownPos[15]);

    lastStartStopTime = now2;
  }

  delay(2);
}


//------------------------------------------------------------
//MOVE FUNCTION AND SEND TO TX

 



void move(int servo, int angle) {
       Serial4.write(char(255));
       Serial4.write(char(servo));
       Serial4.write(char(angle));
}


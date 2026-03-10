#include "ax12.h"
#include "BioloidController.h"


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


// Keep init servo positions in boot and general int array (we're adding and subtracting from these init values with encoder!)
int lastKnownPos[12] = {0, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512};


void setup(){

  Serial.begin(9600);

  //ax12Init(1000000);
  //checkVoltage();

}

void loop(){

 counter = counter + 1;
  if (counter >= timeout) {
    
    if (debug == true) {Serial.println("No connection!");}

    // SET motors and servos into neutral position and update array
    lastKnownPos[1] = 512;
    lastKnownPos[2] = 512;
    lastKnownPos[3] = 512;
    lastKnownPos[4] = 512;
    lastKnownPos[5] = 512;

    lastKnownPos[6] = 512;
    lastKnownPos[7] = 512;
    lastKnownPos[8] = 512;
    lastKnownPos[9] = 512;
    lastKnownPos[10] = 512;

    SetPosition(2 , lastKnownPos[1]);
    SetPosition(3 , lastKnownPos[2]);
    SetPosition(4 , lastKnownPos[3]);
    SetPosition(5 , lastKnownPos[4]);
    SetPosition(6 , lastKnownPos[5]);

    SetPosition(7 , lastKnownPos[6]);
    SetPosition(8 , lastKnownPos[7]);
    SetPosition(9 , lastKnownPos[8]);
    SetPosition(10 , lastKnownPos[9]);
    SetPosition(11 , lastKnownPos[10]);

  }


//====================================================================
// ---------------------| SERIAL RADIO CONTROL |----------------------
//====================================================================

  if (Serial.available() > 2) {

   counter = 0; // Reset connection timeout counter
   startbyte = Serial.read();
    
    if (startbyte == 255) {
      for (i = 0; i < 2; i++) {
        userInput[i] = Serial.read();
      }
     servo = userInput[0];
     pos = userInput[1];
     if (pos == 255) {
       servo = 255;
     }

      switch (servo) {
        case 1: // SERVO 1
          if (debug == true) {Serial.print("servo1: ");Serial.println(pos);}
          pos = map(pos, 0, 180, 200, 800);
          SetPosition(2 , pos);
          lastKnownPos[1] = pos;
          break;
        case 2: // SERVO 2
           if (debug == true) {Serial.print("servo2: ");Serial.println(pos);}
           pos = map(pos, 0, 180, 200, 800);
          SetPosition(3 , pos);
          lastKnownPos[2] = pos;
          break;
        case 3: // SERVO 3
          if (debug == true) {Serial.print("servo3: ");Serial.println(pos);}
           pos = map(pos, 0, 180, 200, 800);
          SetPosition(4 , pos);
          lastKnownPos[3] = pos;
          break;
        case 4: // SERVO 4
          if (debug == true) {Serial.print("servo4: ");Serial.println(pos);}
           pos = map(pos, 0, 180, 200, 800);
          SetPosition(5 , pos);
          lastKnownPos[4] = pos;
          break;
        case 5: // SERVO 5
          if (debug == true) {Serial.print("servo5: ");Serial.println(pos);}
           pos = map(pos, 0, 180, 200, 800);
          SetPosition(6 , pos);
          lastKnownPos[5] = pos;
          break;
        case 6: // SERVO 6
          if (debug == true) {Serial.print("servo6: ");Serial.println(pos);}
           pos = map(pos, 0, 180, 200, 800);
          SetPosition(7 , pos);
          lastKnownPos[6] = pos;
          break;
        case 7: // SERVO 7
          if (debug == true) {Serial.print("servo7: ");Serial.println(pos);}
           pos = map(pos, 0, 180, 200, 800);
          SetPosition(8 , pos);
          lastKnownPos[7] = pos;
          break;
        case 8: // SERVO 8
          if (debug == true) {Serial.print("servo8: ");Serial.println(pos);}
           pos = map(pos, 0, 180, 200, 800);
          SetPosition(9 , pos);
          lastKnownPos[8] = pos;
          break;
        case 9: // SERVO 9
          if (debug == true) {Serial.print("servo9: ");Serial.println(pos);}
           pos = map(pos, 0, 180, 200, 800);
          SetPosition(10 , pos);
          lastKnownPos[9] = pos;
          break;
        case 10: // SERVO 10
          if (debug == true) {Serial.print("servo10: ");Serial.println(pos);}
           pos = map(pos, 0, 180, 200, 800);
          SetPosition(11 , pos);
          lastKnownPos[10] = pos;
          break;
        }
      }
    }
    delay(2);
}



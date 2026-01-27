/*

=========================================================================
VWR ROBOTICS CORNFLAKE 2025 - SABRE V2 - HEAVY ROBOT
=========================================================================
-------------------------------------------------------------------------
CORNFLAKE RECEIVER V2.3 FOR SABRE V2 based on TEENSY 4.0
-------------------------------------------------------------------------
Hardware used for this sketch:

- Teensy 4.0
- HC-12 433MHz Serial Trancievers (ON 3V!)
- Sabretooth 2x25A Motor Controller
- APA102 Leds (Adafruit Dotstar)  (Run Teensy on 72MHz!)
- 20A BEC                         (input 12V/ Output 5V)
- 2x 5A LiPo batteries 12V        (12V 10A)
- 4x 5A LiPo batteries 24V        (24V 10A)

------------------------------------------------------------------------    

Telemetry Array Config:

0-(0/1/2)         Remote to Robot communication status
1-(VALUE)         FWD/REV
2-(VALUE)         TURN
3-(VALUE)         TILT
4-(VALUE)         PAN
5-(VALUE)         PAN2 (joy dial)
6-(0/1/2/3/4/5)   LIGHTS
7-(0/1)           SERVO MODE
8-(0/1)           FIRE/SHOOT
9-(VALUE)         TEMP
10-(VALUE)        DOF1
11-(VALUE)        DOF2
12-(VALUE)        DOF3
13-(VALUE)        GYRO
14-(VALUE)        BATT
15-(0/1)          CRITICAL ERROR

*/

// SABERTOOTH PACKET SERIAL VIA LIB
#include <Sabertooth.h>
Sabertooth ST(128, Serial1); // The Sabertooth is on address 128. We'll name its object ST.

int runningFlag = 0;

// DOTSTAR APA102
#include <Adafruit_DotStar.h>
#define NUMPIXELS 48
#define DATAPIN    19
#define CLOCKPIN   18
Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);

int lidarToggle = 0;

// SSR
#define SSRpin1 8           // ssr 1
#define SSRpin2 9           // ssr 2

int SSR1 = 0;
int SSR2 = 0;

int sequence = 0;

int LEDinterval = 80;
int LEDinterval2 = 50;

unsigned long now = millis();
unsigned long lastStartStopTime = 0;

// Pulse APA102
int pulseLed = 800;
int RGBwarning = 10;
int RGBpolice; 
int RGBpolice2; 
int pulsespeed = 300;
int lightState = 0;

// Status led remote connection
static int serialLed = 23;

// HC12 SERIAL INPUT SERVO/VALUE
int userInput[3];    // raw input from serial buffer, 3 bytes
int startbyte;       // start byte, begin reading input
int servo;           // which servo to pulse?
int pos;             // servo angle 0-180
int i;               // iterator

// Serial timeout on lost connection
const long timeout = 600;
long counter = 0;

// Debug on/off
boolean debug = false;
boolean busyflag = false;
boolean warningFlashers = false;

// COLOR SETS
uint32_t colorwhite = 0xFFFFFF;
//uint32_t colorred = 0xFF0000;
uint32_t colorredoff = 0x300000;
uint32_t colorblue = 0x0000FF;
uint32_t colorblueoff = 0x000039;
uint32_t coloryellow = 0xFFFF00;
uint32_t coloryellowoff = 0x302d00;
uint32_t colorgreen = 0x00dc1f;
uint32_t coloroff = 0x000000;
uint32_t colorred = 0xb40101;
uint32_t colorreddark = 0x120000;
uint32_t colorreddarker = 0x090000;
uint32_t off = 0x000000;

// Keep init servo positions in boot and general int array (we're adding and subtracting from these init values with encoder!)
int lastKnownPos[13] = {0,90,90,180,90,90,0,0,0,0,10,100,90};

int framerate = 100; // intro led speed lulz


void setup() {

// PORTS
  Serial.begin(9600);   // debug monitor
  Serial1.begin(9600);  // SABERTOOTH (TX)
  Serial2.begin(9600);  // HC12 C044 (RX) (AUTONOM/ENCRYPTION MODULE)
  //Serial3.begin(9600);  // RASPBERRY PI (TX) (CANON CONTROLLER)

  pinMode(SSRpin1, OUTPUT);
  pinMode(SSRpin2, OUTPUT);

// SABERTOOTH
  ST.autobaud(); // Send the autobaud command to the Sabertooth controller(s).
  ST.drive(0); // Sabertooth Packed Serial FWD
  ST.turn(0); // Sabertooth Packed Serial TURN

  // PREVENT RUNAWAY ROBOT ON SIGNAL TIMEOUT FROM CONTROLLER OR MC REBOOT
  ST.setTimeout(950);


    if (debug == true){Serial.println("CORNFLAKE SABRE V2.3!");}

    strip.begin();
    strip.show();

    pinMode(serialLed, OUTPUT); // serial connection remote status led
    digitalWrite(serialLed, 0);
   // digitalWrite(errorLed, 0);

    digitalWrite(SSRpin1, 0);
    digitalWrite(SSRpin2, 0);

    ledBoot();
            
     if (debug == true){Serial.println("READY!");}

} 

void loop() {

  now = millis();

  // STATE MACHINE

  if ((lastStartStopTime + (LEDinterval)) < now) {
      sequence = sequence + 1;
      //Serial.println(sequence);
       if (sequence > 7){
              sequence = 0;
       }
      lastStartStopTime = now;
    }

    RGBwarning = 128+127*cos(2*PI/pulseLed*now);
    RGBpolice = 128+127*cos(2*PI/pulsespeed*now);
    RGBpolice2 = 128+127*cos(2*PI/pulsespeed*now+255);
  
 // Serial safety - Full stop on lost connection of controller.
     counter = counter + 1;
     
      if (counter >= timeout){
          if (debug == true){Serial.println("No connection!");}

            // SET motors and servos into neutral position and update array
            ST.drive(0); // Sabertooth Packed Serial
            ST.turn(0); // Sabertooth Packed Serial

            // Show YELLOW WARNING LIGHTS
            warningFlashers = true;
            digitalWrite(serialLed, 0);
            
          }
        

// Start reading sender(s)

    if (Serial2.available() > 2) {

       startbyte = Serial2.read();
       counter = 0; // Reset connection timeout counter
       
        warningFlashers = false;
       digitalWrite(serialLed, 1);
    
       if (startbyte == 255) {

          for (i=0;i<2;i++){ userInput[i] = Serial2.read();}

            servo = userInput[0];
            pos = userInput[1];
            if (pos == 255){ servo = 255;}

            offFront();

           if (lightState == 0){
             // OFF
            } else if (lightState == 1){
                strip.setPixelColor(0, 255,255,255);
                strip.setPixelColor(1, 255,255,255);
      
                strip.setPixelColor(19, 255,255,255);
                strip.setPixelColor(20, 255,255,255);

            
            } else if (lightState == 2){
              
                strip.setPixelColor(0, 255,255,255);
                strip.setPixelColor(1, 255,255,255);
                strip.setPixelColor(2, 255,255,255);
                strip.setPixelColor(3, 255,255,255);

                strip.setPixelColor(17, 255,255,255);
                strip.setPixelColor(18, 255,255,255);
                strip.setPixelColor(19, 255,255,255);
                strip.setPixelColor(20, 255,255,255);

            
            } else if (lightState == 3){
              
                strip.setPixelColor(0, 255,255,255);
                strip.setPixelColor(1, 255,255,255);
                strip.setPixelColor(2, 255,255,255);
                strip.setPixelColor(3, 255,255,255);
                strip.setPixelColor(4, 255,255,255);
                strip.setPixelColor(5, 255,255,255);
                strip.setPixelColor(6, 255,255,255);

                strip.setPixelColor(14, 255,255,255);
                strip.setPixelColor(15, 255,255,255);
                strip.setPixelColor(16, 255,255,255);
                strip.setPixelColor(17, 255,255,255);
                strip.setPixelColor(18, 255,255,255);
                strip.setPixelColor(19, 255,255,255);
                strip.setPixelColor(20, 255,255,255);

            
            } else if (lightState == 4){

      
                strip.setPixelColor(0, 255,255,255);
                strip.setPixelColor(1, 255,255,255);
      
                strip.setPixelColor(19, 255,255,255);
                strip.setPixelColor(20, 255,255,255);
         
                // Police light pulse override
                strip.setPixelColor(2, 0,0,RGBpolice);
                strip.setPixelColor(3, 0,0,RGBpolice);
                strip.setPixelColor(4, 0,0,RGBpolice);
                strip.setPixelColor(5, 0,0,RGBpolice);
      
                strip.setPixelColor(15, 0,0,RGBpolice2);
                strip.setPixelColor(16, 0,0,RGBpolice2);
                strip.setPixelColor(17, 0,0,RGBpolice2);
                strip.setPixelColor(18, 0,0,RGBpolice2);

            }
         
         switch (servo) {
            case 1: // DRIVE
              if (debug == true){Serial.print("FWD/REV: ");Serial.println(pos);}
              
              offRear();

               if (pos > 98){ 
                      strip.setPixelColor(21, 255,0,0);
                      strip.setPixelColor(22, 255,0,0);
                      strip.setPixelColor(23, 255,0,0);
                      strip.setPixelColor(24, 255,0,0);
          
                      strip.setPixelColor(38, 255,0,0);
                      strip.setPixelColor(39, 255,0,0);
                      strip.setPixelColor(40, 255,0,0);
                      strip.setPixelColor(41, 255,0,0);
                      
                      
                    } else if (pos < 80){
                      strip.setPixelColor(21, 255,0,0);
                      strip.setPixelColor(22, 255,0,0);
                      strip.setPixelColor(23, 255,0,0);
                      strip.setPixelColor(24, 255,255,255);
                      strip.setPixelColor(25, 255,255,255);
                      strip.setPixelColor(26, 255,255,255);
          
                      strip.setPixelColor(36, 255,255,255);
                      strip.setPixelColor(37, 255,255,255);
                      strip.setPixelColor(38, 255,255,255);
                      strip.setPixelColor(39, 255,0,0);
                      strip.setPixelColor(40, 255,0,0);
                      strip.setPixelColor(41, 255,0,0);
                     
                    } else {
                      strip.setPixelColor(21, 255,0,0);
                      strip.setPixelColor(22, 255,0,0);
                      strip.setPixelColor(23, 255,0,0);
                      strip.setPixelColor(24, 255,0,0);
                      strip.setPixelColor(25, 255,0,0);
                      strip.setPixelColor(26, 255,0,0);
                      strip.setPixelColor(27, 255,0,0);
                      strip.setPixelColor(28, 255,0,0);
          
                      strip.setPixelColor(34, 255,0,0);
                      strip.setPixelColor(35, 255,0,0);
                      strip.setPixelColor(36, 255,0,0);
                      strip.setPixelColor(37, 255,0,0);
                      strip.setPixelColor(38, 255,0,0);
                      strip.setPixelColor(39, 255,0,0);
                      strip.setPixelColor(40, 255,0,0);
                      strip.setPixelColor(41, 255,0,0);
                     
                    }
               
               pos = map(pos, 0, 180, -127, 127);
               ST.drive(pos);
               lastKnownPos[1] = pos;
               break;
            case 2: // TURN
              if (debug == true){Serial.print("TURN: ");Serial.println(pos);}
              pos = map(pos, 0, 180, 127, -127);
              ST.turn(pos);
              lastKnownPos[2] = pos;
               break;
            case 3: // TILT
              if (debug == true){Serial.print("TILT: ");Serial.println(pos);}
               break;
            case 4: // PAN
              if (debug == true){Serial.print("PAN: ");Serial.println(pos);}
               break;
            case 5: // PAN 2 (rotation stick)
               break;
            case 6: // LIGHTS
                if (pos == 0) {
                   if (lightState != 0) {
                       lightState = 0; 
                    } 
                    
                 } else if (pos == 1) {
                   if (lightState != 1) {
                       lightState = 1; 
                    }
                    
                 } else if (pos == 2){
                  if (lightState != 2) {
                      lightState = 2;
                    }
                 
                 } else if (pos == 3) {
                  if (lightState != 3) {
                      lightState = 3;
                    }
                    
                 } else if (pos == 4) {
                   if (lightState != 4) {
                       lightState = 4;
                    }
                 }
                 lastKnownPos[6] = pos;
                break; 
             case 7: // SERVO MODE/LIDAR SWITCH/GAIT CHANGE
                if (pos == 0) {
                    digitalWrite(SSRpin1, 0);
                 } else if (pos == 1) {
                    digitalWrite(SSRpin1, 1);
                 }
                 lastKnownPos[7] = pos;
                break;
             case 8: // FIRE BUTTON
                if (pos == 0) {
                     digitalWrite(SSRpin2, 0);
                     busyflag = 0;
                 } else if (pos == 1) {
                    digitalWrite(SSRpin2, 1);
                    
                 }
                lastKnownPos[8] = pos;
                break; 
         }
      }            
    }
    
// led flashers
if (warningFlashers == true){
    if (sequence == 0){

                strip.setPixelColor(0, 255,255,0);
                strip.setPixelColor(1, 255,255,0);
                strip.setPixelColor(2, 255,255,0);
                strip.setPixelColor(3, 255,255,0);

                strip.setPixelColor(17, 0,0,0);
                strip.setPixelColor(18, 0,0,0);
                strip.setPixelColor(19, 0,0,0);
                strip.setPixelColor(20, 0,0,0);


                strip.setPixelColor(21, 0,0,0);
                strip.setPixelColor(22, 0,0,0);
                strip.setPixelColor(23, 0,0,0);
                strip.setPixelColor(24, 0,0,0);
    
                strip.setPixelColor(38, 255,255,0);
                strip.setPixelColor(39, 255,255,0);
                strip.setPixelColor(40, 255,255,0);
                strip.setPixelColor(41, 255,255,0);


          }else if (sequence == 1){

                strip.setPixelColor(0, 0,0,0);
                strip.setPixelColor(1, 0,0,0);
                strip.setPixelColor(2, 0,0,0);
                strip.setPixelColor(3, 0,0,0);

                strip.setPixelColor(17, 0,0,0);
                strip.setPixelColor(18, 0,0,0);
                strip.setPixelColor(19, 0,0,0);
                strip.setPixelColor(20, 0,0,0);


                strip.setPixelColor(21, 0,0,0);
                strip.setPixelColor(22, 0,0,0);
                strip.setPixelColor(23, 0,0,0);
                strip.setPixelColor(24, 0,0,0);
    
                strip.setPixelColor(38, 0,0,0);
                strip.setPixelColor(39, 0,0,0);
                strip.setPixelColor(40, 0,0,0);
                strip.setPixelColor(41, 0,0,0);

            }else if (sequence == 2){


                strip.setPixelColor(0, 255,255,0);
                strip.setPixelColor(1, 255,255,0);
                strip.setPixelColor(2, 255,255,0);
                strip.setPixelColor(3, 255,255,0);

                strip.setPixelColor(17, 0,0,0);
                strip.setPixelColor(18, 0,0,0);
                strip.setPixelColor(19, 0,0,0);
                strip.setPixelColor(20, 0,0,0);


                strip.setPixelColor(21, 0,0,0);
                strip.setPixelColor(22, 0,0,0);
                strip.setPixelColor(23, 0,0,0);
                strip.setPixelColor(24, 0,0,0);
    
                strip.setPixelColor(38, 255,255,0);
                strip.setPixelColor(39, 255,255,0);
                strip.setPixelColor(40, 255,255,0);
                strip.setPixelColor(41, 255,255,0);

            }else if (sequence == 3){

                strip.setPixelColor(0, 0,0,0);
                strip.setPixelColor(1, 0,0,0);
                strip.setPixelColor(2, 0,0,0);
                strip.setPixelColor(3, 0,0,0);

                strip.setPixelColor(17, 0,0,0);
                strip.setPixelColor(18, 0,0,0);
                strip.setPixelColor(19, 0,0,0);
                strip.setPixelColor(20, 0,0,0);


                strip.setPixelColor(21, 0,0,0);
                strip.setPixelColor(22, 0,0,0);
                strip.setPixelColor(23, 0,0,0);
                strip.setPixelColor(24, 0,0,0);
    
                strip.setPixelColor(38, 0,0,0);
                strip.setPixelColor(39, 0,0,0);
                strip.setPixelColor(40, 0,0,0);
                strip.setPixelColor(41, 0,0,0);

          }else if (sequence == 4){




                strip.setPixelColor(0, 0,0,0);
                strip.setPixelColor(1, 0,0,0);
                strip.setPixelColor(2, 0,0,0);
                strip.setPixelColor(3, 0,0,0);

                strip.setPixelColor(17, 255,255,0);
                strip.setPixelColor(18, 255,255,0);
                strip.setPixelColor(19, 255,255,0);
                strip.setPixelColor(20, 255,255,0);


                strip.setPixelColor(21, 255,255,0);
                strip.setPixelColor(22, 255,255,0);
                strip.setPixelColor(23, 255,255,0);
                strip.setPixelColor(24, 255,255,0);
    
                strip.setPixelColor(38, 0,0,0);
                strip.setPixelColor(39, 0,0,0);
                strip.setPixelColor(40, 0,0,0);
                strip.setPixelColor(41, 0,0,0);

          }else if (sequence == 5){

                strip.setPixelColor(0, 0,0,0);
                strip.setPixelColor(1, 0,0,0);
                strip.setPixelColor(2, 0,0,0);
                strip.setPixelColor(3, 0,0,0);

                strip.setPixelColor(17, 0,0,0);
                strip.setPixelColor(18, 0,0,0);
                strip.setPixelColor(19, 0,0,0);
                strip.setPixelColor(20, 0,0,0);


                strip.setPixelColor(21, 0,0,0);
                strip.setPixelColor(22, 0,0,0);
                strip.setPixelColor(23, 0,0,0);
                strip.setPixelColor(24, 0,0,0);
    
                strip.setPixelColor(38, 0,0,0);
                strip.setPixelColor(39, 0,0,0);
                strip.setPixelColor(40, 0,0,0);
                strip.setPixelColor(41, 0,0,0);

  

            }else if (sequence == 6){




                strip.setPixelColor(0, 0,0,0);
                strip.setPixelColor(1, 0,0,0);
                strip.setPixelColor(2, 0,0,0);
                strip.setPixelColor(3, 0,0,0);

                strip.setPixelColor(17, 255,255,0);
                strip.setPixelColor(18, 255,255,0);
                strip.setPixelColor(19, 255,255,0);
                strip.setPixelColor(20, 255,255,0);


                strip.setPixelColor(21, 255,255,0);
                strip.setPixelColor(22, 255,255,0);
                strip.setPixelColor(23, 255,255,0);
                strip.setPixelColor(24, 255,255,0);
    
                strip.setPixelColor(38, 0,0,0);
                strip.setPixelColor(39, 0,0,0);
                strip.setPixelColor(40, 0,0,0);
                strip.setPixelColor(41, 0,0,0);

          }else if (sequence == 7){

                strip.setPixelColor(0, 0,0,0);
                strip.setPixelColor(1, 0,0,0);
                strip.setPixelColor(2, 0,0,0);
                strip.setPixelColor(3, 0,0,0);

                strip.setPixelColor(17, 0,0,0);
                strip.setPixelColor(18, 0,0,0);
                strip.setPixelColor(19, 0,0,0);
                strip.setPixelColor(20, 0,0,0);


                strip.setPixelColor(21, 0,0,0);
                strip.setPixelColor(22, 0,0,0);
                strip.setPixelColor(23, 0,0,0);
                strip.setPixelColor(24, 0,0,0);
    
                strip.setPixelColor(38, 0,0,0);
                strip.setPixelColor(39, 0,0,0);
                strip.setPixelColor(40, 0,0,0);
                strip.setPixelColor(41, 0,0,0);

            
            }

    } //end warning flashers bool

    strip.show();
    delay(2);
} 




void offFront(){
          strip.setPixelColor(0, 0,0,0);
          strip.setPixelColor(1, 0,0,0);
          strip.setPixelColor(2, 0,0,0);
          strip.setPixelColor(3, 0,0,0);
          strip.setPixelColor(4, 0,0,0);
          strip.setPixelColor(5, 0,0,0);
          strip.setPixelColor(6, 0,0,0);
          strip.setPixelColor(7, 0,0,0);
          strip.setPixelColor(8, 0,0,0);
          strip.setPixelColor(9, 0,0,0);
          strip.setPixelColor(10, 0,0,0);
          strip.setPixelColor(11, 0,0,0);
          strip.setPixelColor(12, 0,0,0);
          strip.setPixelColor(13, 0,0,0);
          strip.setPixelColor(14, 0,0,0);
          strip.setPixelColor(15, 0,0,0);
          strip.setPixelColor(16, 0,0,0);
          strip.setPixelColor(17, 0,0,0);
          strip.setPixelColor(18, 0,0,0);
          strip.setPixelColor(19, 0,0,0);
          strip.setPixelColor(20, 0,0,0);
         

}
void offRear(){

            strip.setPixelColor(21, 0,0,0);
            strip.setPixelColor(22, 0,0,0);
            strip.setPixelColor(23, 0,0,0);
            strip.setPixelColor(24, 0,0,0);
            strip.setPixelColor(25, 0,0,0);
            strip.setPixelColor(26, 0,0,0);
            strip.setPixelColor(27, 0,0,0);
            strip.setPixelColor(28, 0,0,0);
            strip.setPixelColor(29, 0,0,0);
            strip.setPixelColor(30, 0,0,0);
            strip.setPixelColor(31, 0,0,0);
            strip.setPixelColor(32, 0,0,0);
            strip.setPixelColor(33, 0,0,0);
            strip.setPixelColor(34, 0,0,0);
            strip.setPixelColor(35, 0,0,0);
            strip.setPixelColor(36, 0,0,0);
            strip.setPixelColor(37, 0,0,0);
            strip.setPixelColor(38, 0,0,0);
            strip.setPixelColor(39, 0,0,0);
            strip.setPixelColor(40, 0,0,0);
            strip.setPixelColor(41, 0,0,0);

}


//------------------------------------------------------------
// TELEMETRY SEND TO TX 2
//------------------------------------------------------------

void telemetry(int item, int data) {
//       Serial2.write(char(255));
//       Serial2.write(char(lastKnownPos[0]));
//       Serial2.write(char(lastKnownPos[1]));
//       Serial2.write(char(lastKnownPos[2]));
//       Serial2.write(char(lastKnownPos[3]));
//       Serial2.write(char(lastKnownPos[4]));
//       Serial2.write(char(lastKnownPos[5]));
//       Serial2.write(char(lastKnownPos[6]));
//       Serial2.write(char(lastKnownPos[7]));
//       Serial2.write(char(lastKnownPos[8]));
//       Serial2.write(char(lastKnownPos[9]));
//       Serial2.write(char(lastKnownPos[10]));
//       Serial2.write(char(lastKnownPos[11]));

}



void ledBoot(){
   // Wait for other stuff to boot (lulz)

          strip.setPixelColor(0, 0,0,0);
          strip.setPixelColor(1, 0,0,0);
          strip.setPixelColor(2, 0,0,0);
          strip.setPixelColor(3, 0,0,0);
          strip.setPixelColor(4, 0,0,0);
          strip.setPixelColor(5, 0,0,0);
          strip.setPixelColor(6, 0,0,0);
          strip.setPixelColor(7, 0,0,0);
          strip.setPixelColor(8, 0,0,0);
          strip.setPixelColor(9, 0,0,0);
          strip.setPixelColor(10, 0,0,0);
          strip.setPixelColor(11, 0,0,0);
          strip.setPixelColor(12, 0,0,0);
          strip.setPixelColor(13, 0,0,0);
          strip.setPixelColor(14, 0,0,0);
          strip.setPixelColor(15, 0,0,0);
          strip.setPixelColor(16, 0,0,0);
          strip.setPixelColor(17, 0,0,0);
          strip.setPixelColor(18, 0,0,0);
          strip.setPixelColor(19, 0,0,0);
          strip.setPixelColor(20, 0,0,0);

            strip.setPixelColor(21, 0,0,0);
            strip.setPixelColor(22, 0,0,0);
            strip.setPixelColor(23, 0,0,0);
            strip.setPixelColor(24, 0,0,0);
            strip.setPixelColor(25, 0,0,0);
            strip.setPixelColor(26, 0,0,0);
            strip.setPixelColor(27, 0,0,0);
            strip.setPixelColor(28, 0,0,0);
            strip.setPixelColor(29, 0,0,0);
            strip.setPixelColor(30, 0,0,0);
            strip.setPixelColor(31, 0,0,0);
            strip.setPixelColor(32, 0,0,0);
            strip.setPixelColor(33, 0,0,0);
            strip.setPixelColor(34, 0,0,0);
            strip.setPixelColor(35, 0,0,0);
            strip.setPixelColor(36, 0,0,0);
            strip.setPixelColor(37, 0,0,0);
            strip.setPixelColor(38, 0,0,0);
            strip.setPixelColor(39, 0,0,0);
            strip.setPixelColor(40, 0,0,0);
            strip.setPixelColor(41, 0,0,0);

            strip.show();
            delay(500); 
          
            strip.setPixelColor(0, 255,0,0);
          strip.setPixelColor(1, 0,0,0);
          strip.setPixelColor(2, 0,0,0);
          strip.setPixelColor(3, 0,0,0);
          strip.setPixelColor(4, 0,0,0);
          strip.setPixelColor(5, 0,0,0);
          strip.setPixelColor(6, 0,0,0);
          strip.setPixelColor(7, 0,0,0);
          strip.setPixelColor(8, 0,0,0);
          strip.setPixelColor(9, 0,0,0);
          strip.setPixelColor(10, 0,0,0);
          strip.setPixelColor(11, 0,0,0);
          strip.setPixelColor(12, 0,0,0);
          strip.setPixelColor(13, 0,0,0);
          strip.setPixelColor(14, 0,0,0);
          strip.setPixelColor(15, 0,0,0);
          strip.setPixelColor(16, 0,0,0);
          strip.setPixelColor(17, 0,0,0);
          strip.setPixelColor(18, 0,0,0);
          strip.setPixelColor(19, 0,0,0);
          strip.setPixelColor(20, 255,0,0);

            strip.setPixelColor(21, 255,0,0);
            strip.setPixelColor(22, 0,0,0);
            strip.setPixelColor(23, 0,0,0);
            strip.setPixelColor(24, 0,0,0);
            strip.setPixelColor(25, 0,0,0);
            strip.setPixelColor(26, 0,0,0);
            strip.setPixelColor(27, 0,0,0);
            strip.setPixelColor(28, 0,0,0);
            strip.setPixelColor(29, 0,0,0);
            strip.setPixelColor(30, 0,0,0);
            strip.setPixelColor(31, 0,0,0);
            strip.setPixelColor(32, 0,0,0);
            strip.setPixelColor(33, 0,0,0);
            strip.setPixelColor(34, 0,0,0);
            strip.setPixelColor(35, 0,0,0);
            strip.setPixelColor(36, 0,0,0);
            strip.setPixelColor(37, 0,0,0);
            strip.setPixelColor(38, 0,0,0);
            strip.setPixelColor(39, 0,0,0);
            strip.setPixelColor(40, 0,0,0);
            strip.setPixelColor(41, 255,0,0);

            strip.show();
            delay(500); 
         
            strip.setPixelColor(0, 255,0,0);
          strip.setPixelColor(1, 0,0,0);
          strip.setPixelColor(2, 0,0,0);
          strip.setPixelColor(3, 0,0,0);
          strip.setPixelColor(4, 0,0,0);
          strip.setPixelColor(5, 0,0,0);
          strip.setPixelColor(6, 0,0,0);
          strip.setPixelColor(7, 0,0,0);
          strip.setPixelColor(8, 0,0,0);
          strip.setPixelColor(9, 0,0,0);
          strip.setPixelColor(10, 0,0,0);
          strip.setPixelColor(11, 0,0,0);
          strip.setPixelColor(12, 0,0,0);
          strip.setPixelColor(13, 0,0,0);
          strip.setPixelColor(14, 0,0,0);
          strip.setPixelColor(15, 0,0,0);
          strip.setPixelColor(16, 0,0,0);
          strip.setPixelColor(17, 0,0,0);
          strip.setPixelColor(18, 0,0,0);
          strip.setPixelColor(19, 0,0,0);
          strip.setPixelColor(20, 255,0,0);

            strip.setPixelColor(21, 255,0,0);
            strip.setPixelColor(22, 255,0,0);
            strip.setPixelColor(23, 0,0,0);
            strip.setPixelColor(24, 0,0,0);
            strip.setPixelColor(25, 0,0,0);
            strip.setPixelColor(26, 0,0,0);
            strip.setPixelColor(27, 0,0,0);
            strip.setPixelColor(28, 0,0,0);
            strip.setPixelColor(29, 0,0,0);
            strip.setPixelColor(30, 0,0,0);
            strip.setPixelColor(31, 0,0,0);
            strip.setPixelColor(32, 0,0,0);
            strip.setPixelColor(33, 0,0,0);
            strip.setPixelColor(34, 0,0,0);
            strip.setPixelColor(35, 0,0,0);
            strip.setPixelColor(36, 0,0,0);
            strip.setPixelColor(37, 0,0,0);
            strip.setPixelColor(38, 0,0,0);
            strip.setPixelColor(39, 0,0,0);
            strip.setPixelColor(40, 255,0,0);
            strip.setPixelColor(41, 255,0,0);
  
            strip.show();
            delay(500); 

                       strip.setPixelColor(0, 255,0,0);
          strip.setPixelColor(1, 255,0,0);
          strip.setPixelColor(2, 255,0,0);
          strip.setPixelColor(3, 0,0,0);
          strip.setPixelColor(4, 0,0,0);
          strip.setPixelColor(5, 0,0,0);
          strip.setPixelColor(6, 0,0,0);
          strip.setPixelColor(7, 0,0,0);
          strip.setPixelColor(8, 0,0,0);
          strip.setPixelColor(9, 0,0,0);
          strip.setPixelColor(10, 0,0,0);
          strip.setPixelColor(11, 0,0,0);
          strip.setPixelColor(12, 0,0,0);
          strip.setPixelColor(13, 0,0,0);
          strip.setPixelColor(14, 0,0,0);
          strip.setPixelColor(15, 0,0,0);
          strip.setPixelColor(16, 0,0,0);
          strip.setPixelColor(17, 0,0,0);
          strip.setPixelColor(18, 255,0,0);
          strip.setPixelColor(19, 255,0,0);
          strip.setPixelColor(20, 255,0,0);

            strip.setPixelColor(21, 255,0,0);
            strip.setPixelColor(22, 255,0,0);
            strip.setPixelColor(23, 255,0,0);
            strip.setPixelColor(24, 0,0,0);
            strip.setPixelColor(25, 0,0,0);
            strip.setPixelColor(26, 0,0,0);
            strip.setPixelColor(27, 0,0,0);
            strip.setPixelColor(28, 0,0,0);
            strip.setPixelColor(29, 0,0,0);
            strip.setPixelColor(30, 0,0,0);
            strip.setPixelColor(31, 0,0,0);
            strip.setPixelColor(32, 0,0,0);
            strip.setPixelColor(33, 0,0,0);
            strip.setPixelColor(34, 0,0,0);
            strip.setPixelColor(35, 0,0,0);
            strip.setPixelColor(36, 0,0,0);
            strip.setPixelColor(37, 0,0,0);
            strip.setPixelColor(38, 0,0,0);
            strip.setPixelColor(39, 255,0,0);
            strip.setPixelColor(40, 255,0,0);
            strip.setPixelColor(41, 255,0,0);
  
            strip.show();
            delay(500); 

             strip.setPixelColor(0, 255,0,0);
          strip.setPixelColor(1, 255,0,0);
          strip.setPixelColor(2, 255,0,0);
          strip.setPixelColor(3, 255,0,0);
          strip.setPixelColor(4, 0,0,0);
          strip.setPixelColor(5, 0,0,0);
          strip.setPixelColor(6, 0,0,0);
          strip.setPixelColor(7, 0,0,0);
          strip.setPixelColor(8, 0,0,0);
          strip.setPixelColor(9, 0,0,0);
          strip.setPixelColor(10, 0,0,0);
          strip.setPixelColor(11, 0,0,0);
          strip.setPixelColor(12, 0,0,0);
          strip.setPixelColor(13, 0,0,0);
          strip.setPixelColor(14, 0,0,0);
          strip.setPixelColor(15, 0,0,0);
          strip.setPixelColor(16, 0,0,0);
          strip.setPixelColor(17, 255,0,0);
          strip.setPixelColor(18, 255,0,0);
          strip.setPixelColor(19, 255,0,0);
          strip.setPixelColor(20, 255,0,0);

            strip.setPixelColor(21, 255,0,0);
            strip.setPixelColor(22, 255,0,0);
            strip.setPixelColor(23, 255,0,0);
            strip.setPixelColor(24, 255,0,0);
            strip.setPixelColor(25, 0,0,0);
            strip.setPixelColor(26, 0,0,0);
            strip.setPixelColor(27, 0,0,0);
            strip.setPixelColor(28, 0,0,0);
            strip.setPixelColor(29, 0,0,0);
            strip.setPixelColor(30, 0,0,0);
            strip.setPixelColor(31, 0,0,0);
            strip.setPixelColor(32, 0,0,0);
            strip.setPixelColor(33, 0,0,0);
            strip.setPixelColor(34, 0,0,0);
            strip.setPixelColor(35, 0,0,0);
            strip.setPixelColor(36, 0,0,0);
            strip.setPixelColor(37, 0,0,0);
            strip.setPixelColor(38, 255,0,0);
            strip.setPixelColor(39, 255,0,0);
            strip.setPixelColor(40, 255,0,0);
            strip.setPixelColor(41, 255,0,0);
  
            strip.show();
            delay(500); 

            strip.setPixelColor(0, 255,0,0);
          strip.setPixelColor(1, 255,0,0);
          strip.setPixelColor(2, 255,0,0);
          strip.setPixelColor(3, 255,0,0);
          strip.setPixelColor(4, 255,0,0);
          strip.setPixelColor(5, 0,0,0);
          strip.setPixelColor(6, 0,0,0);
          strip.setPixelColor(7, 0,0,0);
          strip.setPixelColor(8, 0,0,0);
          strip.setPixelColor(9, 0,0,0);
          strip.setPixelColor(10, 0,0,0);
          strip.setPixelColor(11, 0,0,0);
          strip.setPixelColor(12, 0,0,0);
          strip.setPixelColor(13, 0,0,0);
          strip.setPixelColor(14, 0,0,0);
          strip.setPixelColor(15, 0,0,0);
          strip.setPixelColor(16, 255,0,0);
          strip.setPixelColor(17, 255,0,0);
          strip.setPixelColor(18, 255,0,0);
          strip.setPixelColor(19, 255,0,0);
          strip.setPixelColor(20, 255,0,0);

            strip.setPixelColor(21, 255,0,0);
            strip.setPixelColor(22, 255,0,0);
            strip.setPixelColor(23, 255,0,0);
            strip.setPixelColor(24, 255,0,0);
            strip.setPixelColor(25, 255,0,0);
            strip.setPixelColor(26, 0,0,0);
            strip.setPixelColor(27, 0,0,0);
            strip.setPixelColor(28, 0,0,0);
            strip.setPixelColor(29, 0,0,0);
            strip.setPixelColor(30, 0,0,0);
            strip.setPixelColor(31, 0,0,0);
            strip.setPixelColor(32, 0,0,0);
            strip.setPixelColor(33, 0,0,0);
            strip.setPixelColor(34, 0,0,0);
            strip.setPixelColor(35, 0,0,0);
            strip.setPixelColor(36, 0,0,0);
            strip.setPixelColor(37, 255,0,0);
            strip.setPixelColor(38, 255,0,0);
            strip.setPixelColor(39, 255,0,0);
            strip.setPixelColor(40, 255,0,0);
            strip.setPixelColor(41, 255,0,0);
  
            strip.show();
            delay(500); 

            strip.setPixelColor(0, 255,0,0);
          strip.setPixelColor(1, 255,0,0);
          strip.setPixelColor(2, 255,0,0);
          strip.setPixelColor(3, 255,0,0);
          strip.setPixelColor(4, 255,0,0);
          strip.setPixelColor(5, 255,0,0);
          strip.setPixelColor(6, 0,0,0);
          strip.setPixelColor(7, 0,0,0);
          strip.setPixelColor(8, 0,0,0);
          strip.setPixelColor(9, 0,0,0);
          strip.setPixelColor(10, 0,0,0);
          strip.setPixelColor(11, 0,0,0);
          strip.setPixelColor(12, 0,0,0);
          strip.setPixelColor(13, 0,0,0);
          strip.setPixelColor(14, 0,0,0);
          strip.setPixelColor(15, 255,0,0);
          strip.setPixelColor(16, 255,0,0);
          strip.setPixelColor(17, 255,0,0);
          strip.setPixelColor(18, 255,0,0);
          strip.setPixelColor(19, 255,0,0);
          strip.setPixelColor(20, 255,0,0);

            strip.setPixelColor(21, 255,0,0);
            strip.setPixelColor(22, 255,0,0);
            strip.setPixelColor(23, 255,0,0);
            strip.setPixelColor(24, 255,0,0);
            strip.setPixelColor(25, 255,0,0);
            strip.setPixelColor(26, 255,0,0);
            strip.setPixelColor(27, 0,0,0);
            strip.setPixelColor(28, 0,0,0);
            strip.setPixelColor(29, 0,0,0);
            strip.setPixelColor(30, 0,0,0);
            strip.setPixelColor(31, 0,0,0);
            strip.setPixelColor(32, 0,0,0);
            strip.setPixelColor(33, 0,0,0);
            strip.setPixelColor(34, 0,0,0);
            strip.setPixelColor(35, 0,0,0);
            strip.setPixelColor(36, 255,0,0);
            strip.setPixelColor(37, 255,0,0);
            strip.setPixelColor(38, 255,0,0);
            strip.setPixelColor(39, 255,0,0);
            strip.setPixelColor(40, 255,0,0);
            strip.setPixelColor(41, 255,0,0);
  
            strip.show();
            delay(500); 

            strip.setPixelColor(0, 255,0,0);
          strip.setPixelColor(1, 255,0,0);
          strip.setPixelColor(2, 255,0,0);
          strip.setPixelColor(3, 255,0,0);
          strip.setPixelColor(4, 255,0,0);
          strip.setPixelColor(5, 255,0,0);
          strip.setPixelColor(6, 255,0,0);
          strip.setPixelColor(7, 0,0,0);
          strip.setPixelColor(8, 0,0,0);
          strip.setPixelColor(9, 0,0,0);
          strip.setPixelColor(10, 0,0,0);
          strip.setPixelColor(11, 0,0,0);
          strip.setPixelColor(12, 0,0,0);
          strip.setPixelColor(13, 0,0,0);
          strip.setPixelColor(14, 255,0,0);
          strip.setPixelColor(15, 255,0,0);
          strip.setPixelColor(16, 255,0,0);
          strip.setPixelColor(17, 255,0,0);
          strip.setPixelColor(18, 255,0,0);
          strip.setPixelColor(19, 255,0,0);
          strip.setPixelColor(20, 255,0,0);

            strip.setPixelColor(21, 255,0,0);
            strip.setPixelColor(22, 255,0,0);
            strip.setPixelColor(23, 255,0,0);
            strip.setPixelColor(24, 255,0,0);
            strip.setPixelColor(25, 255,0,0);
            strip.setPixelColor(26, 255,0,0);
            strip.setPixelColor(27, 255,0,0);
            strip.setPixelColor(28, 0,0,0);
            strip.setPixelColor(29, 0,0,0);
            strip.setPixelColor(30, 0,0,0);
            strip.setPixelColor(31, 0,0,0);
            strip.setPixelColor(32, 0,0,0);
            strip.setPixelColor(33, 0,0,0);
            strip.setPixelColor(34, 0,0,0);
            strip.setPixelColor(35, 255,0,0);
            strip.setPixelColor(36, 255,0,0);
            strip.setPixelColor(37, 255,0,0);
            strip.setPixelColor(38, 255,0,0);
            strip.setPixelColor(39, 255,0,0);
            strip.setPixelColor(40, 255,0,0);
            strip.setPixelColor(41, 255,0,0);
  
            strip.show();
            delay(500); 

            strip.setPixelColor(0, 255,0,0);
          strip.setPixelColor(1, 255,0,0);
          strip.setPixelColor(2, 255,0,0);
          strip.setPixelColor(3, 255,0,0);
          strip.setPixelColor(4, 255,0,0);
          strip.setPixelColor(5, 255,0,0);
          strip.setPixelColor(6, 255,0,0);
          strip.setPixelColor(7, 255,0,0);
          strip.setPixelColor(8, 0,0,0);
          strip.setPixelColor(9, 0,0,0);
          strip.setPixelColor(10, 0,0,0);
          strip.setPixelColor(11, 0,0,0);
          strip.setPixelColor(12, 0,0,0);
          strip.setPixelColor(13, 255,0,0);
          strip.setPixelColor(14, 255,0,0);
          strip.setPixelColor(15, 255,0,0);
          strip.setPixelColor(16, 255,0,0);
          strip.setPixelColor(17, 255,0,0);
          strip.setPixelColor(18, 255,0,0);
          strip.setPixelColor(19, 255,0,0);
          strip.setPixelColor(20, 255,0,0);

            strip.setPixelColor(21, 255,0,0);
            strip.setPixelColor(22, 255,0,0);
            strip.setPixelColor(23, 255,0,0);
            strip.setPixelColor(24, 255,0,0);
            strip.setPixelColor(25, 255,0,0);
            strip.setPixelColor(26, 255,0,0);
            strip.setPixelColor(27, 255,0,0);
            strip.setPixelColor(28, 255,0,0);
            strip.setPixelColor(29, 0,0,0);
            strip.setPixelColor(30, 0,0,0);
            strip.setPixelColor(31, 0,0,0);
            strip.setPixelColor(32, 0,0,0);
            strip.setPixelColor(33, 0,0,0);
            strip.setPixelColor(34, 255,0,0);
            strip.setPixelColor(35, 255,0,0);
            strip.setPixelColor(36, 255,0,0);
            strip.setPixelColor(37, 255,0,0);
            strip.setPixelColor(38, 255,0,0);
            strip.setPixelColor(39, 255,0,0);
            strip.setPixelColor(40, 255,0,0);
            strip.setPixelColor(41, 255,0,0);
  
            strip.show();
            delay(500); 

            strip.setPixelColor(0, 255,0,0);
          strip.setPixelColor(1, 255,0,0);
          strip.setPixelColor(2, 255,0,0);
          strip.setPixelColor(3, 255,0,0);
          strip.setPixelColor(4, 255,0,0);
          strip.setPixelColor(5, 255,0,0);
          strip.setPixelColor(6, 255,0,0);
          strip.setPixelColor(7, 255,0,0);
          strip.setPixelColor(8, 255,0,0);
          strip.setPixelColor(9, 0,0,0);
          strip.setPixelColor(10, 0,0,0);
          strip.setPixelColor(11, 0,0,0);
          strip.setPixelColor(12, 255,0,0);
          strip.setPixelColor(13, 255,0,0);
          strip.setPixelColor(14, 255,0,0);
          strip.setPixelColor(15, 255,0,0);
          strip.setPixelColor(16, 255,0,0);
          strip.setPixelColor(17, 255,0,0);
          strip.setPixelColor(18, 255,0,0);
          strip.setPixelColor(19, 255,0,0);
          strip.setPixelColor(20, 255,0,0);

            strip.setPixelColor(21, 255,0,0);
            strip.setPixelColor(22, 255,0,0);
            strip.setPixelColor(23, 255,0,0);
            strip.setPixelColor(24, 255,0,0);
            strip.setPixelColor(25, 255,0,0);
            strip.setPixelColor(26, 255,0,0);
            strip.setPixelColor(27, 255,0,0);
            strip.setPixelColor(28, 255,0,0);
            strip.setPixelColor(29, 255,0,0);
            strip.setPixelColor(30, 0,0,0);
            strip.setPixelColor(31, 0,0,0);
            strip.setPixelColor(32, 0,0,0);
            strip.setPixelColor(33, 255,0,0);
            strip.setPixelColor(34, 255,0,0);
            strip.setPixelColor(35, 255,0,0);
            strip.setPixelColor(36, 255,0,0);
            strip.setPixelColor(37, 255,0,0);
            strip.setPixelColor(38, 255,0,0);
            strip.setPixelColor(39, 255,0,0);
            strip.setPixelColor(40, 255,0,0);
            strip.setPixelColor(41, 255,0,0);
  
            strip.show();
            delay(500); 

            //systemready();
}


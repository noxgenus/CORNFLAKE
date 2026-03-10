/*
=========================================================================
VWR ROBOTICS CORNFLAKE 2024 - MULTI JOYSTICK CONTROLLER BOX
=========================================================================
-------------------------------------------------------------------------
CORNFLAKE TRANSMITTER BOX V3.0
-------------------------------------------------------------------------
Hardware used for this sketch:

- Teensy V4.0
- 2X HC-12 433MHz Serial Tranciever
- APA102 Leds (Adafruit Dotstar)
- Analog Joystick (Servocity)
- Adafruit Analog joypad
- Push buttons
- 10k pots

CONTROL CHANNEL: HC-12 #1 CHANNEL = C044
TELEMETRY CHANNEL: HC-12 #2 CHANNEL = C066

HC-12 SET CHANNEL: AT+C044

COMMS:

move(#)

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

                
    1-drive               int joypin1 = A0; //Joystick X1
    2-steer               int joypin2 = A1; //Joystick Y1
    3-?int                int joypin3 = A2; //Joystick top rotate
    4-tilt                int joypin3 = A3; //Joystick X1 tilt
    5-pan                 int joypin4 = A4; //Joystick Y1 pan
    6-shoulder L          int dofpin1 = A5; // POT shoulder L
    7-arm L               int dofpin1 = A6; // POT arm L
    8-shoulder R          int dofpin1 = A7; // POT shoulder R
    9-arm R               int dofpin1 = A8; // POT arm R
    10-gripper         L  int dofpin1 = A9; // POT grip L
    11-gripper R          int dofpin1 = A10; // POT grip R (NOT IN USE!)
    12-rc led controller  int // POT rc led xontrol


*/

#include <SPI.h>

#include <Adafruit_DotStar.h>
#define NUMPIXELS 4 
#define DATAPIN    3
#define CLOCKPIN   2
Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);



// COLOR SETS

uint32_t colorwhite = 0xFFFFFF;
uint32_t colorred = 0xFF0000;
uint32_t colorredoff = 0x300000;
uint32_t colorblue = 0x0000FF;
uint32_t colorblueoff = 0x000039;
uint32_t coloryellow = 0xFFFF00;
uint32_t coloryellowoff = 0x302d00;
uint32_t colorgreen = 0x00dc1f;
uint32_t coloroff = 0x000000;

// ANALOG INPUTS

// Joystick
const int joypin1 = A0; //Joystick X1
const int joypin2 = A1; //Joystick Y1


// Joypad
const int joypin3 = A3; //Joystick X1
const int joypin4 = A10; //Joystick Y1

// Top dial
const int joypin5 = A2; //Joystick Y2

// DOF POTS
const int dofpin1 = A11; // POT
const int dofpin2 = A6; // POT
const int dofpin3 = A7; // POT

const int dofpin4 = A8; // POT
const int dofpin5 = A9; // POT

//const int dofpin6 = A10; // POT
//const int dofpin7 = A11; // POT
//const int dofpin8 = A12; // POT
//const int dofpin9 = A13; // POT


// Buttons

const int blueButton = 6;
const int yellowButton = 5;
const int redButton = 4;
const int joyButton = 7;


// JOYSTICK BUTTON PRESS LIGHT CONTROL
int count = 0;
int buttonState = 0; 
int lastButtonState = 0;  

// JOYSTICK BUTTON PRESS MODE CONTROL
//int count1 = 0;
//int buttonState1 = 0; 
//int lastButtonState1 = 0; 

// JOYSTICK BUTTON PRESS 3rd CONTROL
int count2 = 0;
int buttonState2 = 0; 
int lastButtonState2 = 0; 

// JOYSTICK BUTTON TOP JOYSTICK FIRE CONTROL
int count3 = 0;
int buttonState3 = 0; 
int lastButtonState3 = 0; 

// MODE SWITCH
int modecount = 0;
int modeState = 0; 
int lastModeState = 0;  

// SERVO MODE SWITCH
int servomodecount = 0;
int servomodeState = 0; 
int servolastModeState = 0;  

int servo;
int angle;

int rcLedVal = 90;

int val1 = 90;
int val2 = 90;
int val3 = 90;
int val4 = 90;
int val5 = 90;
int val6 = 90;
int val7 = 90;
int val8 = 90;
int val9 = 90;
int val10 = 90;
int val11 = 90;
int val12 = 90;

int buttonVal1 = 0;
int buttonVal2 = 0;
int buttonVal3 = 0;
int buttonVal4 = 0;

#define DEBOUNCE_INTERVAL 500L // Milliseconds
#define BASE_INTERVAL 500L

unsigned long lastStartStopTime1 = 0;
unsigned long lastStartStopTime2 = 0;
unsigned long lastStartStopTime3 = 0;
unsigned long lastStartStopTime4 = 0;

unsigned long now2 = millis();  // NOW2 FOR DEBOUNCE

void setup() {
  
    Serial.begin(9600);
    Serial1.begin(9600);
    
    
    pinMode(blueButton, INPUT_PULLUP);
    //digitalWrite(lightButton, 1);
    
    pinMode(yellowButton, INPUT_PULLUP);
   // digitalWrite(modeButton, 1);
    
    pinMode(redButton, INPUT_PULLUP);
    //digitalWrite(thirdButton, 1);
    
    pinMode(joyButton, INPUT_PULLUP);
    //digitalWrite(fireButton, 1);

    move(1, val1);    //fwd/rev joy
    move(2, val2);    //steering joy
    move(3, val3);    //? top joy rotate
    move(4, val4);    //tilt joypad
    move(5, val5);    //pan joypad
    move(6, val6);    // ax 1
    move(7, val7);    // ax 2
    move(8, val8);    // ax 3
    move(9, val9);    // ax 4
    move(10, val10);  // ax 5
    move(11, val11);  // ax 6
    move(12, val12);  // ax 7

    move(20, buttonVal1);  //button1
    move(21, buttonVal2);  //button2
    move(22, buttonVal3);  //button3
    move(23, buttonVal4);  //joybutt

    strip.begin();
    strip.setBrightness(64); //set brightness low
    strip.show();

    strip.setPixelColor(0, coloroff);
    strip.setPixelColor(1, coloroff);
    strip.setPixelColor(2, coloroff);
    strip.setPixelColor(3, colorgreen);
    strip.show();
    delay(200);
    strip.setPixelColor(0, coloroff);
    strip.setPixelColor(1, coloroff);
    strip.setPixelColor(2, colorgreen);
    strip.setPixelColor(3, colorgreen);
    strip.show();
    delay(200);
    strip.setPixelColor(0, coloroff);
    strip.setPixelColor(1, colorgreen);
    strip.setPixelColor(2, colorgreen);
    strip.setPixelColor(3, colorgreen);
    strip.show();
    delay(200);
    strip.setPixelColor(0, colorgreen);
    strip.setPixelColor(1, colorgreen);
    strip.setPixelColor(2, colorgreen);
    strip.setPixelColor(3, colorgreen);
    strip.show();
    delay(500);
    strip.setPixelColor(0, colorredoff);
    strip.setPixelColor(1, colorredoff);
    strip.setPixelColor(2, coloryellowoff);
    strip.setPixelColor(3, colorblueoff);
     strip.show();

      delay(500);

}

void loop(){
  

   now2 = millis();

//------------------------------------------------------------
  //READ ANALOG INPUTS TO INT
  val1 = analogRead(joypin1);
  val2 = analogRead(joypin2);
  val3 = analogRead(joypin5);
  val4 = analogRead(joypin3);
  val5 = analogRead(joypin4);
  val6 = analogRead(dofpin1);
  val7 = analogRead(dofpin2);
  val8 = analogRead(dofpin3);
  val9 = analogRead(dofpin4);
  val10 = analogRead(dofpin5);
  val11 = rcLedVal; //(NOT IN USE)
  val12 = rcLedVal;

  //MAP ANALOG JOYSTICK VALS TO SERVO VALS (0-90-180)
  val1 = map(val1, 0, 1023, 180, 0); //reverse pot
  val2 = map(val2, 0, 1023, 180, 0); //reverse pot
  val3 = map(val3, 0, 1023, 0, 180); // joy rotate
  val4 = map(val4, 0, 1023, 0, 180); // pan
  val5 = map(val5, 0, 1023, 0, 180); // tilt

  val6 = map(val6, 0, 1023, 0, 180); // ax12 1
  val7 = map(val7, 0, 1023, 0, 180); // ax12 2
  val8 = map(val8, 0, 1023, 0, 180); // ax12 3
  val9 = map(val9, 0, 1023, 0, 180); // ax12 4
  val10 = map(val10, 0, 1023, 0, 180); // ax12 5
  //val11 = map(val11, 0, 1023, 0, 180);

  // Serial.print("VAL1: ");
  // Serial.print(val1);
  // Serial.print(" VAL2: ");
  // Serial.print(val2);
  // Serial.print(" VAL3: ");
  // Serial.print(val3);
  // Serial.print(" VAL4: ");
  // Serial.print(val4);
  //  Serial.print(" VAL5: ");
  // Serial.print(val5);
  //  Serial.print(" VAL6: ");
  // Serial.print(val6);
  //  Serial.print(" VAL7: ");
  // Serial.print(val7);
  //  Serial.print(" VAL8: ");
  // Serial.print(val8);
  //  Serial.print(" VAL9: ");
  // Serial.print(val9);
  //  Serial.print(" VAL10: ");
  // Serial.print(val10);
  //  Serial.print(" VAL11: ");
  // Serial.print(val11);
  // Serial.print(" VAL12: ");
  // Serial.println(val12);

  // Serial.print(" BLUE: ");
  // Serial.println(buttonVal1);

  // Serial.print(" YEL: ");
  // Serial.println(buttonVal2);

  // Serial.print(" RED: ");
  // Serial.println(buttonVal3);

  // Serial.print(" FIRE: ");
  // Serial.println(buttonVal4);

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
      
              


// ========================================================================
// BUTTONS ================================================================
// ========================================================================


// BLUE BUTTON ------------------------------------------------------------

  if (digitalRead(blueButton) == 1) {
          move(20, 1);
          strip.setPixelColor(3, colorblue);
      } else {
          move(20, 0);  //button1
          strip.setPixelColor(3, colorblueoff);
      }


  // YELLOW BUTTON ------------------------------------------------------------

  if (digitalRead(yellowButton) == 1) {
          move(21, 1);
          strip.setPixelColor(2, coloryellow);
    } else {
          move(21, 0);  //button1
          strip.setPixelColor(2, coloryellowoff);
    }

  

    // RED BUTTON ------------------------------------------------------------

  if (digitalRead(redButton) == 1) {
      move(22, 1);
      strip.setPixelColor(1, colorred);
    } else {
      move(22, 0);
      strip.setPixelColor(1, colorredoff);
  }

// ================================================================
// BUTTON ON TOP JOYSTICK PRESS FOR FIRE

  if (digitalRead(joyButton) == 1) {
        move(23, 1);  //joybutt
        strip.setPixelColor(0, colorred);
      } else {
        move(23, 0);  //joybutt
        strip.setPixelColor(0, coloroff);
  }


 strip.show();
 
// Short delay else loop goes bonkers
  delay(40);
}


//------------------------------------------------------------
//MOVE FUNCTION AND SEND TO TX
void move(int servo, int angle) {
       Serial1.write(char(255));
       Serial1.write(char(servo));
       Serial1.write(char(angle));
}

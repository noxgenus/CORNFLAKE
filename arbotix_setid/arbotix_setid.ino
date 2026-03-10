
#include "ax12.h"
#include "BioloidController.h"


int newID = 5;
int mode = 3; // change id

void setup(){

delay(1000);
ax12SetRegister(0 , mode , newID);
delay(1000);
ax12SetRegister(1 , mode , newID);
delay(1000);
ax12SetRegister(2 , mode , newID);
delay(1000);
ax12SetRegister(3 , mode , newID);
delay(1000);
ax12SetRegister(4 , mode , newID);
delay(1000);
ax12SetRegister(5 , mode , newID);
delay(1000);
ax12SetRegister(6 , mode , newID);
delay(1000);
ax12SetRegister(7 , mode , newID);
delay(1000);
ax12SetRegister(8 , mode , newID);
delay(1000);
ax12SetRegister(9 , mode , newID);
delay(1000);
ax12SetRegister(10 , mode , newID);
delay(1000);
ax12SetRegister(11 , mode , newID);
delay(1000);
ax12SetRegister(12 , mode , newID);
delay(1000);
ax12SetRegister(13 , mode , newID);
delay(1000);
ax12SetRegister(14 , mode , newID);
delay(1000);
ax12SetRegister(15 , mode , newID);
delay(1000);
ax12SetRegister(16 , mode , newID);
delay(1000);
ax12SetRegister(17 , mode , newID);
delay(1000);
ax12SetRegister(18 , mode , newID);
delay(1000);

}

void loop(){
SetPosition(newID , 512);
delay(2000);
SetPosition(newID , 612);
delay(2000);
SetPosition(newID , 412);
delay(8000);
}


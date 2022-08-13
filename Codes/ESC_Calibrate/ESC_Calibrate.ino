//remove propellers when testing

#include <Servo.h>


//PINS
//const PINS with PWM~ for motors
const int flPIN = 3;  //FrontLeft
const int frPIN = 5;  //FrontRight
const int rlPIN = 9;  //RearLeft
const int rrPIN = 6;  //RearRight
const int BUZZER = 8; //BUZZER PIN


//ESCs
Servo ESCfl;    
Servo ESCfr; 
Servo ESCrl; 
Servo ESCrr;



void setup()
{
     //MAX,MIN 1000,2000 attaching esc's
    ESCfl.attach (flPIN,1000,2000);
    ESCfr.attach (frPIN,1000,2000);
       
    ESCrl.attach (rlPIN,1000,2000);
    ESCrr.attach (rrPIN,1000,2000);

    delay(4000);

    int pMIN = 2000;
    ESCfl.write(pMIN);              
    ESCfr.write(pMIN);             
    ESCrl.write(pMIN);     
    ESCrr.write(pMIN);
    
    delay(4000);

    pMIN = 1000;
    ESCfl.write(pMIN);              
    ESCfr.write(pMIN);             
    ESCrl.write(pMIN);     
    ESCrr.write(pMIN);
}

void loop() 
{
  // put your main code here, to run repeatedly:

  int pMIN = 1000;
    ESCfl.write(pMIN);              
    ESCfr.write(pMIN);             
    ESCrl.write(pMIN);     
    ESCrr.write(pMIN);
    

}

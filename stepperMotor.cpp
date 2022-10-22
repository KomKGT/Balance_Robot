#include "Arduino.h"
#include "stepperMotor.h"

steppermotor::steppermotor(int microstep,int directionPinL,int pulsePinL, int directionPinR, int pulsePinR){
    this->directionPinL = directionPinL;
    this->pulsePinL = pulsePinL;
    this->directionPinR = directionPinR;
    this->pulsePinR = pulsePinR;
    this->microstep = microstep;
    pinMode(directionPinL,OUTPUT);
    pinMode(pulsePinL,OUTPUT);
    pinMode(directionPinR,OUTPUT);
    pinMode(pulsePinR,OUTPUT);
    volatile bool  DirForward;
}


//desired_speed not over 350 as stepper speed not lower than 200
void steppermotor::stepperSetspeed(int desired_speed){
    // ใช้เวลากี่วินาทีในการครบ 1 รอบ ; 1 T = 1 pulse
  //  stepperSpeed = 60L * 1000L * 1000L / microstep / desired_speed;
    // ใช้หาว่า 1 cycle ใช้เวลาเท่าไหร่
}

float steppermotor::stepperMove(int motorside, float desired_step){
    
    long start_pulse = 0;
   // long steps_left = abs(desired_step);
    int pulse_state = HIGH;
    int pulse_count = 0;
//    Serial.println(desired_step);
    if(desired_step >= 0){ //direction forward
//      Serial.println("HIGH");// left 0 right 1
    if (motorside == 0) digitalWrite(directionPinL,LOW);
    if (motorside == 1) digitalWrite(directionPinR,HIGH);
      DirForward = true ;
      
      }
     else if(desired_step < 0){ //direction backward
//      Serial.println("LOW");
      if (motorside == 0) digitalWrite(directionPinL,HIGH);
      if (motorside == 1) digitalWrite(directionPinR,LOW);
      DirForward = false ;
      }
      
     desired_step = desired_step *8; //convert to microsteps
    if ((abs(desired_step) < 2.0))  desired_step = 2.0;
    return desired_step;
 
}

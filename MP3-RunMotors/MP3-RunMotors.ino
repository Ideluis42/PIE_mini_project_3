/*Firmware Implementation for the DC Motor Control Mini-Project for PIE
Controller will send speed parameters? TBD
Firmware will read the IR data to determine if robot is within the path limits ; adjust DC motor speeds (PWM) ; create micro corrections whenever IR signal detects the line.*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_MS_PWMServoDriver.h"

#define leftIR 0;
#define rightIR 1;

uint16_t nSpeed;
uint16_t leftIRread;
uint16_t rightIRread;
int defSpeed = 150; 

uint16_t baud = 9600;
uint16_t serialDelay = 20;

Adafruit_MotorShield MS = Adafruit_MotorShield();
Adafruit_DCMotor *LM = MS.getMotor(1);
Adafruit_DCMotor *RM = MS.getMotor(2);
//disp(MS.getMotor(1))
//disp(MS.getMotor(4))
void setup() {
  // put your setup code here, to run once:
  Serial.begin(baud);
  MS.begin();
  LM->setSpeed(defSpeed);
  RM->setSpeed(defSpeed);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() >0){
    nSpeed = Serial.read();
    LM->setSpeed(nSpeed);
    RM->setSpeed(nSpeed); 
    Serial.print("I received: ");
    Serial.println(nSpeed, DEC);
    }
       
  LM->run(FORWARD);
  RM->run(FORWARD);

  delay(1000);

}

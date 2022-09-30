/*Firmware Implementation for the DC Motor Control Mini-Project for PIE
Controller will send speed parameters? TBD
Firmware will read the IR data to determine if robot is within the path limits ; adjust DC motor speeds (PWM) ; create micro corrections whenever IR signal detects the line.*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_MS_PWMServoDriver.h"

#define leftIR 0;
#define rightIR 1;

int defSpeed = 150; 

#define baud 9600;
#define serialDelay 20;

Adafruit_MotorShield MS = Adafruit_MotorShield();
Adafruit_DCMotor *LM = MS.getMotor(1);
Adafruit_DCMotor *RM = MS.getMotor(2);
//disp(MS.getMotor(1))
//disp(MS.getMotor(4))
void setup() {
  // put your setup code here, to run once:
  //Serial.begin(baud);
  MS.begin();
  LM->setSpeed(defSpeed);
  RM->setSpeed(defSpeed);

}

void loop() {
  // put your main code here, to run repeatedly:
  //if(Serial.available >0){
    //nSpeed = Serial.read();
    //nSpeed = nSpeed.toInt();
  //}
  //leftIRread = analogRead(leftIR);
  //rightIRread = analogRead(rightIR);

  LM->run(FORWARD);
  RM->run(FORWARD);

  delay(1000);

}

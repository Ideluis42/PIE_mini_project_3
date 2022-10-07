#include <Wire.h>
#include <Adafruit_MotorShield.h>

int leftIR = 1;
int rightIR = 2;
int leftIRread;
int rightIRread;
int defSpeed = 33;
//uint16_t baud = 9600;

Adafruit_MotorShield MS = Adafruit_MotorShield();
Adafruit_DCMotor *LM = MS.getMotor(1);
Adafruit_DCMotor *RM = MS.getMotor(2);

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(baud);
  MS.begin();
  LM->setSpeed(defSpeed);
  RM->setSpeed(defSpeed);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Read the IR sensors
  leftIRread = analogRead(leftIR);
  rightIRread = analogRead(rightIR);
  //Serial.print(leftIRread);
  //Serial.print(',');
  //Serial.print(rightIRread);
  //Serial.println();
  
  // Detect whether IR is over the line
  if(leftIRread > 750){
    leftIRread = HIGH; // Line detected
  }
  else{
    leftIRread = LOW; // Line not detected
  }
  if(rightIRread >850){
    rightIRread = HIGH; // Line detected
  }
  else{
    rightIRread = LOW; // Line not detected
  }
  // Correct based on which IR sensor is on the line
  if(leftIRread == LOW && rightIRread == LOW){
    LM->run(FORWARD);
    RM->run(FORWARD); 
  }
  if(leftIRread == LOW && rightIRread == HIGH){
    LM->run(FORWARD);
    RM->setSpeed(defSpeed-5);
    RM->run(BACKWARD);
    RM->setSpeed(defSpeed);
  }
  if(leftIRread == HIGH && rightIRread == LOW){
    LM->run(BACKWARD);
    LM->setSpeed(defSpeed-5);
    RM->run(FORWARD);
    LM->setSpeed(defSpeed);
  }
  // Stop because you've finished
  if(leftIRread == HIGH && rightIRread == HIGH){
    LM->run(RELEASE);
    RM->run(RELEASE);
  }
}

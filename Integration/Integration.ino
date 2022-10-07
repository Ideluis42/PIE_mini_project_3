// PIE mini project 3 integration code 
// Last edited 10.7.22

// Include header files
#include <Wire.h>
#include <Adafruit_MotorShield.h>

float currentTime = 0;
float pastTime = 0; 
float elapsedTime = 0;

// PID multipliers
int Kp = 0; // TBD--Proportional Term
int Ki = 0; //TBD--Integral Term
int Kd = 0; //TBD--Derivative Term

int pastError = 0; // Previous error found

int proportional = 0; // instantenous error
int integral = 0; // cumulative error
int derivative = 0; // rate of error

int ctrl = 0;

// motor variables
int initalMotorSpeed = 0; // TBD
int leftMotorSpeed = 0;
int rightMotorSpeed = 0; 
int leftIR = 1;
int rightIR = 2;
uint16_t leftIRread;
uint16_t rightIRread;
#define serialDelay 20;

// define motor shield
Adafruit_MotorShield MS = Adafruit_MotorShield();

// define motors
Adafruit_DCMotor *LM = MS.getMotor(1);
Adafruit_DCMotor *RM = MS.getMotor(2);

int error = 0;
int pidValue = 0;

// Error finding function
// LOW -> Line not detected
// HIGH -> Line detected
int findError() {
  
  // when the robot is centered, both IR sensors should read 0

  // if either of the IR sensors read 1 then whoops there's something wrong

  // pseudocode because idk which pins the IR sensors are in
  // Logic:
  // if left IR sensor reading == 1 and right IR sensor reading == 0
  //    error = -1
  // if left IR sensor reading == 0 and right IR sensor reading == 1
  //    error = 1
  // if left IR sensor reading == 0 and right IR sensor reading = 0
  //    error = 0
  // if both IR sensors reading == 1
  //    error = 2.0

  // return error
  
  leftIRread = analogRead(leftIR);
  rightIRread = analogRead(rightIR);
  
  if(leftIRread > 500){
    leftIRread = HIGH;
  }
  else{
    leftIRread = LOW;
  }
  if(rightIRread >500){
    rightIRread = HIGH;
  }
  else{
    rightIRread = LOW;
  }

  if (leftIRread == HIGH && rightIRread == LOW) {
    return -1; // may need to be one idk just experimenting rn
    }
  if (leftIRread == LOW && rightIRread == HIGH) {
    return 1;
    }
  if (leftIRread == HIGH && rightIR == HIGH) {
    return 2;
    }
  return 0;
  }

// PID controller function
int controller(float error) {
  
  // find elapsed time since last reading
  currentTime = millis();
  elapsedTime = currentTime - pastTime;

  // proportional
  proportional = error;
  // Integration:
  integral = integral + (proportional * currentTime);
  // Derivative: 
  derivative = (proportional - pastError) / currentTime ; 

  // find control value
  ctrl = Kp*proportional + Ki*integral + Kd*derivative;
  // save previous values for next iteration
  pastError = proportional;
  pastTime = currentTime;

  return ctrl; 
  }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // open serial port 
  
  if(Serial.available()) {
    Kp = Serial.read();
    Ki = Serial.read();
    Kd = Serial.read();
    initalMotorSpeed = Serial.read();
    }
    
  MS.begin();
  LM->setSpeed(initalMotorSpeed);
  RM->setSpeed(initalMotorSpeed);
}

void loop() {
  // put your main code here, to run repeatedly:
  // set up motor speed and multipliers
  // read from the serial port 
  // find error:
  error = findError();

  // if we're horribly off course, stop
  if (error == 2.0) {
    LM->run(RELEASE);
    RM->run(RELEASE);
    }

  // find ctrl
  pidValue = controller(error);
  // calc motor speeds -- feel like I'm missing something -- maybe current speed
  // has to factor in somehow
  // LMS = total - init - ctrl
  // RMS = total + init - ctrl 
  leftMotorSpeed = 255 - initalMotorSpeed - pidValue;
  rightMotorSpeed = 255 + initalMotorSpeed - pidValue;
  // set motor speed to calculated values
  LM->setSpeed(leftMotorSpeed);
  RM->setSpeed(rightMotorSpeed);
  
  // run motors?
  LM->run(FORWARD);
  RM->run(FORWARD);
}

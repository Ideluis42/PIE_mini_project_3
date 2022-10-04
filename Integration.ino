// PIE mini project 3 integration code 
// Last edited 10.4.22

// Include header files
#include <Wire.h>
#include <Adafruit_MotorShield.h>

enum states {
  _IDLE, // Pre/Post run state--change with key press?
  _MOVING // Running state
  };
  
states current_state = _IDLE; 

float currentTime = 0;
float pastTime = 0; 
float elapsedTime = 0;

// PID multipliers
float Kp = 0; // TBD--Proportional Term
float Ki = 0; //TBD--Integral Term
float Kd = 0; //TBD--Derivative Term

// desired state
float desiredSensorReading = 0; //TBD with calibration
float pastSensorReading = 0; // Previous Sensor Reading

float pastError = 0; // Previous error found

float proportional = 0; // instantenous error
float integral = 0; // cumulative error
float derivative = 0; // rate of error

float ctrl = 0;

// motor variables
int motorSpeed = 0;

#define leftIR 0;
#define rightIR 1;
#define serialDelay 20;

// define motor shield
Adafruit_MotorShield MS = Adafruit_MotorShield();

// define motors
Adafruit_DCMotor *LM = MS.getMotor(1);
Adafruit_DCMotor *RM = MS.getMotor(2);

// PID controller function
float controller(float sensorReading) {
  
  // find elapsed time since last reading
  currentTime = millis();
  elapsedTime = currentTime - pastTime;

  // proportional
  proportional = desiredSensorReading - sensorReading;
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
  MS.begin();
  LM->setSpeed(motorSpeed);
  RM->setSpeed(motorSpeed);
}

void loop() {
  // put your main code here, to run repeatedly:
  // set up motor speed and multipliers
  // read from the serial port 
  if(Serial.available()) {
    Kp = Serial.parseFloat()
    Ki = Serial.parseFloat()
    Kd = Serial.parseFloat()
    // since motorSpeed is an int, use serial.read
    // if you use Serial.parseFloat it won't save anything 
    motorSpeed = Serial.read()
    }
  // state machine
  switch(current_state)
  {
    case(_IDLE):
    // stop moving, wait for signal to start (button push?)
      break;
    case(_MOVING):
    // moving, correcting position with controller. 
    // signal to end: sensing nothing??
      break;
    }
}

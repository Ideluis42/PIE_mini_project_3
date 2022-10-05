// PIE mini project 3 integration code 
// Last edited 10.5.22

enum states {
  _IDLE, // Pre/Post run state--change with key press?
  _MOVING // Running state
  };
  
states current_state = _IDLE; 

// Include header files
#include <Wire.h>
#include <Adafruit_MotorShield.h>

float currentTime = 0;
float pastTime = 0; 
float elapsedTime = 0;

// PID multipliers
float Kp = 0; // TBD--Proportional Term
float Ki = 0; //TBD--Integral Term
float Kd = 0; //TBD--Derivative Term

// desired state -- difference of the readings of the two sensors wants to be zero
float desiredSensorReading = 0; //TBD with calibration
float pastSensorReading = 0; // Previous Sensor Reading

float pastError = 0; // Previous error found

float proportional = 0; // instantenous error
float integral = 0; // cumulative error
float derivative = 0; // rate of error

float ctrl = 0;

// motor variables
int initalMotorSpeed = 0; // TBD

#define leftIR 0;
#define rightIR 1;
#define serialDelay 20;

// define motor shield
Adafruit_MotorShield MS = Adafruit_MotorShield();

// define motors
Adafruit_DCMotor *LM = MS.getMotor(1);
Adafruit_DCMotor *RM = MS.getMotor(2);

float error = 0.0;
float pidValue = 0.0;
// Error finding function
float findError() {
  // when the robot is centered, both IR sensors should read 0

  // if either of the IR sensors read 1 then whoops there's something wrong

  // pseudocode because idk which pins the IR sensors are in

  // get readings :)
  // if left IR sensor reading == 1 and right IR sensor reading == 0
  //    error = -1
  // if left IR sensor reading == 0 and right IR sensor reading == 1
  //    error = 1
  // if left IR sensor reading == 0 and right IR sensor reading = 0
  //    error = 0
  // if both IR sensors reading == 1
  //    error = 2.0

  // return error
  }

// PID controller function
float controller(float error) {
  
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
    initalMotorSpeed = Serial.read()
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

    // find error

    // if we're horribly off course, stop
    if error == 2.0 {
      current_state = _IDLE;
      break;
      
      }
        
    // find ctrl
    pidValue = controller(error);
    // calc motor speeds
      
      break;
    }
    

}

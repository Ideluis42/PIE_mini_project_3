// Will need to figure out a way to make Kp, Ki, Kd changable
// error = desired_val - actual_val

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
  Serial.begin(9600); // open serial port for debugging :)
}

void loop() {
  // put your main code here, to run repeatedly:

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

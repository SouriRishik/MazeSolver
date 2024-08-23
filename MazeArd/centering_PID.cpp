#include "centering_pid.h"
#include <Arduino.h>
#include <math.h>  // Include this line for the floor function

// PID Controller parameters for Motor 1
float Kp1 = 5.0;
float Ki1 = 0.1;
float Kd1 = 0.05;
float setPoint1 = 5.0; // Desired distance in cm for Motor 1
float previousError1 = 0.0;
float integral_c1 = 0.0;
float derivative1 = 0.0;
float error1 = 0.0;
float output1 = 0.0;
uint32_t prevTime1 = 0;
uint32_t currTime1 = 0;

// Function to compute PID output for Motors
float computePIDMotor(float input, float setPoint1) {
  delay(50);
    	    currTime1 = millis();
    	    uint32_t deltaTime1 = currTime1 - prevTime1;
    	    error1 = setPoint1 - input;
    	    integral_c1 += error1 * deltaTime1;
    	    derivative1 = (error1 - previousError1) / deltaTime1;
    	    output1 = Kp1 * error1 + Ki1 * integral_c1 + Kd1 * derivative1;
    	    previousError1 = error1;
    	    prevTime1 = currTime1;
    	    return output1;
          
}



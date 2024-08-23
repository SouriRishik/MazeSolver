#ifndef CENTERING_PID_H_
#define CENTERING_PID_H_

#include <Arduino.h>

//PID Controller parameters for Motor 1
extern float Kp1;
extern float Ki1;
extern float Kd1;
extern float setPoint1;
extern float previousError1;
extern float integral_c1;
extern float derivative1;
extern float error1;
extern float output1;
extern uint32_t prevTime1;
extern uint32_t currTime1;


// Function prototypes
float computePIDMotor(float input, float setPoint1);

#endif /* CENTERING_PID_H_ */

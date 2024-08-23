#ifndef MOTOR_PID_H_
#define MOTOR_PID_H_

#include <Arduino.h>

// PID parameters for Motor 1
extern float kp1;
extern float ki1;
extern float kd1;
extern float integral1;
extern float previous_error1;
extern uint32_t base_speed1;

// PID parameters for Motor 2
extern float kp2;
extern float ki2;
extern float kd2;
extern float integral2;
extern float previous_error2;
extern uint32_t base_speed2;

extern int encoder0Pos,encoder1Pos;

// Function declarations
float turningPID(float dir, float desired_rpm3);
float computeRPM(uint8_t motor);
void setMotorSpeed(int motor, int pwm_value);
void setMotorSpeed1(int pwm_value);
void computePID1(float desired_rpm1);
void computePID2(float desired_rpm2);

#endif // MOTOR_PID_H_

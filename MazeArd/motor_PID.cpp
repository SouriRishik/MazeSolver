#include "Motor_PID.h"
#include <Arduino.h>
#include "Sensor_Data.h"

const uint8_t MOTOR1_PWM_PIN = 12;
const uint8_t MOTOR2_PWM_PIN = 8;
const uint8_t MOTOR1_PWM_PIN_R =13;
const uint8_t MOTOR2_PWM_PIN_R=9;


// PID parameters for Motor 1
float kp1 = 5.0;
float ki1 = 0.01;
float kd1 = 0.5;
float integral1 = 0.0;
float previous_error1 = 0;
uint32_t base_speed1 = 50;

// PID parameters for Motor 2
float kp2 = 5.0;
float ki2 = 0.01;
float kd2 = 0.5;
float integral2 = 0.0;
float previous_error2 = 0;
uint32_t base_speed2 = 50;

//int encoder0Pos,encoder1Pos;
int ee1 = encoder0Pos;
int ee2 = encoder1Pos;
// Timing
unsigned long lastTime = 0;

// Function to compute the current RPM
float computeRPM(uint8_t motor) {
    //HAL_TIM_Base_Start(&htim1);
    int time_rpm = millis();
    int init_tick1 = 0;
    int init_tick2 = 0;

    if (motor == 1) { // 1 is for right
        while (1) {
            if (time_rpm % 1000 == 0) {
                int tick1 = ee1;
                float right_rpm = (float)(tick1 - init_tick1) / 6800.0;
                return right_rpm;
            }
        }
    } else if (motor == 2) { // 2 is for left
        while (1) {
            if (time_rpm % 1000 == 0) {
                int tick2 = ee2;
                float left_rpm = (float)(tick2 - init_tick2) / 6800.0;
                return left_rpm;
            }
        }
    }

    // If the function reaches this point, something went wrong
    // So we return a default value, e.g., 0.0 or some error indicator
    return 225.0;
}
// Function to set motor speed using PWM
void setMotorSpeed(int motor, int speed) {
    // Ensure speed is within 0 to 255 range
    int pwm = abs(speed);
    delay(50);
    if (motor == 0) { // Motor 1
      if(speed>0)
      {
        analogWrite(MOTOR1_PWM_PIN, pwm);  // Set PWM duty cycle for Motor 1
        analogWrite(MOTOR1_PWM_PIN_R, 0);
      }
      else {
        analogWrite(MOTOR1_PWM_PIN, 0);  // Set PWM duty cycle for Motor 1
        analogWrite(MOTOR1_PWM_PIN_R, pwm);
      }
        //analogWrite(MOTOR1_PWM_PIN_R, pwm);
    }
    else if (motor == 1) { // Motor 2
      if(speed>0)
      {
        analogWrite(MOTOR2_PWM_PIN, pwm);  // Set PWM duty cycle for Motor 1
        analogWrite(MOTOR2_PWM_PIN_R, 0);
      }
      else {
        analogWrite(MOTOR2_PWM_PIN, 0);  // Set PWM duty cycle for Motor 1
        analogWrite(MOTOR2_PWM_PIN_R, pwm);
      }
    }
}

void setMotorSpeed1(int speed)
{
  int pwm = abs(speed);
  analogWrite(MOTOR1_PWM_PIN, pwm);  // Set PWM duty cycle for Motor 1
  analogWrite(MOTOR1_PWM_PIN_R, 0);
  analogWrite(MOTOR2_PWM_PIN, pwm);  // Set PWM duty cycle for Motor 1
  analogWrite(MOTOR2_PWM_PIN_R, 0);
}

// PID control logic to maintain desired RPM for Motor 1
void computePID1(float desired_rpm1) {
    unsigned long currentTime = millis();
    float timechange = (currentTime - lastTime) / 1000.0; // Convert milliseconds to seconds
    float current_rpm1 = computeRPM(0);
    float error1 = desired_rpm1 - current_rpm1;
    integral1 += error1 * timechange;
    float derivative1 = (error1 - previous_error1) / timechange;
    float correction1 = (kp1 * error1 + ki1 * integral1 + kd1 * derivative1);
    correction1 = floor(correction1);
    setMotorSpeed(0, int(base_speed1 + correction1));
    previous_error1 = error1;
    lastTime = currentTime;
    delay(50);
}

// PID control logic to maintain desired RPM for Motor 2
void computePID2(float desired_rpm2) {
    unsigned long currentTime = millis();
    float timechange = (currentTime - lastTime) / 1000.0; // Convert milliseconds to seconds
    float current_rpm2 = computeRPM(1);
    float error2 = desired_rpm2 - current_rpm2;
    integral2 += error2 * timechange;
    float derivative2 = (error2 - previous_error2) / timechange;
    float correction2 = (kp2 * error2 + ki2 * integral2 + kd2 * derivative2);
    correction2 = floor(correction2);
    setMotorSpeed(1, base_speed2 + correction2);
    previous_error2 = error2;
    lastTime = currentTime;
    delay(50);
}

//Parameters for PID_Turnung
float kp3 = 5.0;
float ki3 = 0.01;
float kd3 = 0.5;
float integral3 = 0.0;
float previous_error3 = 0.0;
uint32_t base_speed3 = 50;
int time=0;


float turningPID(float dir, float desired_rpm3) // dir is for right or left, 1 for right, 2 for left
{
	while(1)
  {
		if(dir==1)
		{
									float timechange = millis() - time;
									float current_rpm3 = computeRPM(1);
									float error3 = desired_rpm3 - current_rpm3;
									integral3 += error3;
									float derivative3 = error3 - previous_error3;
									float correction3 = (kp3 * error3 + ki3 * integral3 + kd3 * derivative3) * timechange;
									correction3 = floor(correction3);
									setMotorSpeed(1, base_speed2 + correction3);
									setMotorSpeed(0, -(base_speed3+ correction3));
									previous_error3 = error3;
									time = millis();
                  delay(50);

		}
		else if(dir==2)
		{
									float timechange = millis() - time;
								    float current_rpm3 = computeRPM(1);
								    float error3 = desired_rpm3 - current_rpm3;
								    integral3 += error3;
								    float derivative3 = error3 - previous_error3;
								    float correction3 = (kp3 * error3 + ki3 * integral3 + kd3 * derivative3) * timechange;
								    correction3 = floor(correction3);
								    setMotorSpeed(0, base_speed3 + correction3);
								    setMotorSpeed(1, -(base_speed3+ correction3));
								    previous_error3 = error3;
								    time = millis();
                    delay(50);
		}
	}
}


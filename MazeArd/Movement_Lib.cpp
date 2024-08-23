#include "Movement_Lib.h"
#include "centering_PID.h"
#include "motor_PID.h"
#include "Sensor_Data.h"
#include <Arduino.h>
#include "MPU6050_6Axis_MotionApps20.h"


// External variable declarations
float distance1, distance2, distance3;
//int encoder0Pos,encoder1Pos;
float e1= encoder0Pos;
float e2 = encoder1Pos;
//float ypr[3];
float left_dist=distance2;
float right_dist=distance1;
float front_dist=distance3;
float tick_1rev=6800.0;


void move_fwd()
{
  
  RobotControl robot;
  /*
  float left_dist=distance2;
  float right_dist=distance1;
  float front_dist=distance3;
	float init_tick = (e1+e2)/2; //tick reading from encoder comes here
  */
  
	float delta_tick=0.0;
	while((delta_tick)<=(1.8*tick_1rev))
	{
        //robot.run();
        //robot.UltraSonic(); 
        float init_tick = (e1+e2);
		    float US_left, US_right,US_front,rpm;
		    US_left = robot.UltraSonic2();
		    US_right = robot.UltraSonic1();
        US_front=robot.UltraSonic3();
        // Serial.println("Calculating Yaw...");
        // float yaw=robot.CalculateYaw();//yaw values come here
        if(US_front<5.0)
        break;
		    if(US_left<10.0 && US_right<10.0 )
		    {
			    rpm = computePIDMotor((US_right-US_left)/2,0);//centering PID
		    }
		    if(US_left<10.0 && US_right>10.0)
		    {
		    	rpm = computePIDMotor(US_left, 5);
		    }
		    if(US_right<10.0 && US_left>10.0)
		    {
		    	rpm = computePIDMotor(US_right, -5);
		    }
		    if(US_right>10.0 && US_left>10.0)
		    {
		    	rpm = computePIDMotor(0, 0);
		    }
		    //float rpm1 = computePIDMotor(US_left);
		    computePID1(rpm);//RPM Matching loops
		    computePID2(rpm);
		    int tick=(e1+e2); //tick reading from encoder comes here
		    delta_tick = tick-init_tick;
	}
}
// Function to turn the robot right
void turn_right()
{
  RobotControl robot;
  //robot.run();
  float yaw;
	float init_yaw=0.0;
	float delta_yaw = 0.0;
		while(delta_yaw<90.0)
		{
			yaw=robot.CalculateYaw();//yaw values come here
      //robot.getYPR(yaw);
			turningPID(2, 120);
			//yaw=ypr[0];
			delta_yaw = yaw-init_yaw;
      if(delta_yaw>90.0)
      break;
		}
}

void turn_left()
{
  RobotControl robot;
  //robot.run();
  float yaw;
	float init_yaw=0.0;
	float delta_yaw = 0.0;
		while(delta_yaw<90.0)
		{
			yaw = robot.CalculateYaw();//yaw values come here
      //robot.getYPR(yaw);
			turningPID(1, 120);
			//yaw=ypr[0];
			delta_yaw = yaw-init_yaw;
		}
}
// Function to turn the robot right

void reset()
 {
    setMotorSpeed(0, 0); // Stop Motor 1
    setMotorSpeed(1, 0); // Stop Motor 2
    delay(15000);
}

bool iswall_left()
 {
    RobotControl robot; 
        //robot.run();
        //robot.UltraSonic(); 
		    float d2 = robot.UltraSonic2();
    return (d2 < 7.5);
}

bool iswall_right()
 {
        RobotControl robot;
        // robot.run();
        // robot.UltraSonic(); 
      float d1 = robot.UltraSonic1();
    return (d1 < 7.5);
}
bool iswall_front()
 {
    RobotControl robot;

        // robot.run();
        // robot.UltraSonic(); 
        float d3 = robot.UltraSonic3();
        return (d3 < 7.5);
}

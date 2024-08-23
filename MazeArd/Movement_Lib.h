#ifndef MOVEMENT_LIB_H
#define MOVEMENT_LIB_H

#include "Movement_Lib.h"
#include "centering_pid.h"
#include "motor_PID.h"
#include "Sensor_Data.h"
#include <Arduino.h>

// External variable declarations
extern float distance1, distance2, distance3;
//extern int encoder0Pos, encoder1Pos;
extern float e1;
extern float e2;
extern float ypr[3];
extern float left_dist;
extern float right_dist;
extern float front_dist;
extern float tick_1rev;

// Function prototypes
void move_fwd();
void turn_left();
void turn_right();
void reset();
bool iswall_left();
bool iswall_right();
bool iswall_front();

#endif // MOVEMENT_LIB_H

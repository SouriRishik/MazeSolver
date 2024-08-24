#ifndef IMU_H
#define IMU_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// External variables
extern MPU6050 mpu;
extern bool blinkState;
extern bool dmpReady;
extern uint8_t mpuIntStatus;
extern uint8_t devStatus;
extern uint16_t packetSize;
extern uint16_t fifoCount;
extern uint8_t fifoBuffer[64];
extern Quaternion q;
extern VectorInt16 aa;
extern VectorInt16 aaReal;
extern VectorInt16 aaWorld;
extern VectorFloat gravity;
extern float euler[3];
extern float ypr[3];
extern uint8_t teapotPacket[14];
extern volatile bool mpuInterrupt;

// Function prototypes
void imu_init();
float imu_run();
void dmpDataReady();

#endif // IMU_H

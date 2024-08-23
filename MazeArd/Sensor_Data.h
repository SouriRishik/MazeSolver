#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

class RobotControl {
public:
    void initialize(); // Initializes the system
    //void run();        // Executes the main control loop
    void dmpDataReady();
    void handleSwitch();
    void handleSwitch1();
    void UltraSonic();
    //void getYPR(float yaw);
    long UltraSonic1();
    long UltraSonic2();
    long UltraSonic3();
    float CalculateYaw();
    static RobotControl* instance; // Static instance pointer


private:
    // Global variables
    int val;
    int encoder0PinA = 3;
    int encoder0PinB = 4;
    int encoder0Pos = 0;
    int encoder0PinALast = LOW;
    int n = LOW;

    int yaw;

    int encoder1PinA = 2;
    int encoder1PinB = 5;
    int encoder1Pos = 0;
    int encoder1PinALast = LOW;
    int n1 = LOW;

    int trigPin1 = A3;
    int echoPin1 = A4;

    int trigPin2 = A1;
    int echoPin2 = A2;

    int trigPin3 = A5;
    int echoPin3 = A6;
    const byte switchPin = 2;

    #define INTERRUPT_PIN 2 
    #define INTERRUPT_PIN1 3 
    #define LED_PIN 13 
    bool blinkState = false;

    // MPU control/status vars
    bool dmpReady = false;  
    uint8_t mpuIntStatus;   
    uint8_t devStatus;      
    uint16_t packetSize;    
    uint16_t fifoCount;     
    uint8_t fifoBuffer[64]; 

    Quaternion q;           
    VectorInt16 aa;         
    VectorInt16 aaReal;     
    VectorInt16 aaWorld;    
    VectorFloat gravity;    
    float euler[3];         
    float ypr[3];           

    uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

    volatile bool mpuInterrupt = false;

    MPU6050 mpu;
};

#endif // SENSOR_DATA_H

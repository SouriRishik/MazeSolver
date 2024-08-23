#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Sensor_Data.h"

RobotControl* RobotControl::instance = nullptr;

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

int trigPin3 =A5;
int echoPin3 = A6;
const byte switchPin = 2;

#define INTERRUPT_PIN 2 
#define INTERRUPT_PIN1 3 

#define INTERRUPT_EN 18
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

void RobotControl::initialize() {
    // Initialize serial communication
    Serial.begin(9600);
    while (!Serial);

    // Initialize I2C and MPU6050
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(500000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // Initialize MPU6050
    mpu.initialize();

    pinMode(INTERRUPT_PIN, INPUT);
    
    // Setup encoder pins
    pinMode(encoder0PinA, INPUT_PULLUP);
    pinMode(encoder0PinB, INPUT_PULLUP);
    pinMode(encoder1PinA, INPUT_PULLUP);
    pinMode(encoder1PinB, INPUT_PULLUP);
    
    // Setup ultrasonic pins
    pinMode(trigPin1, OUTPUT);
    pinMode(echoPin1, INPUT);
    pinMode(trigPin2, OUTPUT);
    pinMode(echoPin2, INPUT);
    pinMode(trigPin3, OUTPUT);
    pinMode(echoPin3, INPUT);
    pinMode(switchPin, INPUT_PULLUP);

    instance = this; // Store the current instance

    attachInterrupt(digitalPinToInterrupt(encoder0PinA), []() { instance->handleSwitch(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder1PinA), []() { instance->handleSwitch1(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), []() { instance->dmpDataReady(); }, RISING);
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), RISING);

    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
      Serial.println("dev Status 0");
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

// void RobotControl::run() {
//     while (true) {
//         UltraSonic();
//         if (!dmpReady) continue;

//         if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
//             #ifdef OUTPUT_READABLE_YAWPITCHROLL
//                 mpu.dmpGetQuaternion(&q, fifoBuffer);
//                 mpu.dmpGetGravity(&gravity, &q);
//                 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//                 Serial.print("4");
//                 Serial.println(ypr[0] * 180 / M_PI);
//                 yaw = ypr[0] * 180 / M_PI;
//             #endif

//         }
//     }
// } 

float RobotControl::CalculateYaw()
{
  Serial.println("Calculating Yaw...");
    if (dmpReady)
    {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            #ifdef OUTPUT_READABLE_YAWPITCHROLL
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                Serial.println("4 ");
                Serial.println(ypr[0] * 180 / M_PI);
                yaw = ypr[0] * 180 / M_PI;
            #endif

        }
    }
    return yaw;
}

//Different code for IMU
// float RobotControl::CalculateYaw()
// {
// if (!dmpReady) continue;

//         if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
//             #ifdef OUTPUT_READABLE_YAWPITCHROLL
//                 mpu.dmpGetQuaternion(&q, fifoBuffer);
//                 mpu.dmpGetGravity(&gravity, &q);
//                 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//                 Serial.print("4");
//                 Serial.println(ypr[0] * 180 / M_PI);
//                 yaw = ypr[0] * 180 / M_PI;

//             #endif
//             blinkState = !blinkState;
//             digitalWrite(LED_PIN, blinkState);
//             return yaw;

//         }
//     }
// void RobotControl::getYPR(float yaw) {
//     yaw = ypr[0];
    
// }


void RobotControl::dmpDataReady() {
    mpuInterrupt = true;
}
 
long RobotControl::UltraSonic1() {
    long duration1, distance1;
    digitalWrite(trigPin1, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);
    duration1 = pulseIn(echoPin1, HIGH);
    distance1 = (duration1 / 2) / 29.1;

    if (distance1 >= 500 || distance1 <= 0) {
        Serial.println("Out of range1");
    } else {
        Serial.print("1 ");
        Serial.println(distance1);
    }
    return distance1;
}

long RobotControl::UltraSonic2()
{
    long duration2, distance2;
    digitalWrite(trigPin2, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);
    duration2 = pulseIn(echoPin2, HIGH);
    distance2 = (duration2 / 2) / 29.1;

    if (distance2 >= 500 || distance2 <= 0) {
        Serial.println("Out of range");
    } else {
         Serial.print("2 ");
         Serial.println(distance2);
    }
    return distance2;
}

long RobotControl::UltraSonic3()
{
    
    long duration3, distance3;
    digitalWrite(trigPin3, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin3, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin3, LOW);
    duration3 = pulseIn(echoPin3, HIGH);
    distance3 = (duration3 / 2) / 29.1;

    if (distance3 >= 500 || distance3 <= 0) {
        Serial.println("Out of range3");
    } else {
         Serial.print("3 ");
         Serial.println(distance3);
    }
    return distance3;
}

void RobotControl::handleSwitch() {
    n = digitalRead(encoder0PinA);
    if ((encoder0PinALast == LOW) && (n == HIGH)) {
        if (digitalRead(encoder0PinB) == LOW) {
            encoder0Pos--;
        } else {
            encoder0Pos++;
        }
         Serial.print("5 ");
         Serial.println(encoder0Pos);
    }
    encoder0PinALast = n;
}

void RobotControl::handleSwitch1() {
    n1 = digitalRead(encoder1PinA);
    if ((encoder1PinALast == LOW) && (n1 == HIGH)) {
        if (digitalRead(encoder1PinB) == LOW) {
            encoder1Pos--;
        } else {
            encoder1Pos++;
        }
         Serial.print("6 ");
         Serial.println(encoder1Pos);
    }
    encoder1PinALast = n1;
}

#ifndef MAIN_H_
#define MAIN_H_

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <ArduinoSTL.h>
#include <Vector.h>

// Pin Definitions
extern int encoder0PinA;
extern int encoder0PinB;
extern int trigPin1;
extern int echoPin1;
extern int trigPin2;
extern int echoPin2;
extern int trigPin3;
extern int echoPin3;
extern const byte switchPin;

// Global Variables
extern int yaw;
extern bool blinkState;
extern volatile bool mpuInterrupt;
extern bool dmpReady;
extern uint8_t mpuIntStatus;
extern uint8_t devStatus;
extern uint16_t packetSize;
extern uint16_t fifoCount;
extern uint8_t fifoBuffer[64];
extern uint8_t teapotPacket[14];
extern int encoder0Pos;
extern int encoder0PinALast;
extern int n;

// MPU6050 Object
extern MPU6050 mpu;

// Orientation/Motion Variables
extern Quaternion q;
extern VectorInt16 aa;
extern VectorInt16 aaReal;
extern VectorInt16 aaWorld;
extern VectorFloat gravity;
extern float euler[3];
extern float ypr[3];

// Function Declarations
void initializeGPIO();
void initializeTimers();
void led();
void handleSwitch();
void errorHandler();
void dmpDataReady();
void start();

// Function Declarations for Ultrasonic Sensor
int UltraSonic();

// Structures
typedef struct coor {
    int row;
    int col;
    int value;
} coord;

typedef struct cell_infos {
    bool walls[4];
    bool visited;
    int angle_update;
    bool dead;
} cell_info;

typedef struct wall_mazes {
    cell_info cells[16][16];
} wall_maze;

extern wall_maze maze;

// Movement Definitions
#define UP 0
#define DOWN 1
#define LEFT 2
#define RIGHT 3

const int rows = 16;
const int cols = 16;

// Queue and Stack Structs
struct Node {
    coord data;
    Node* next;
};

struct Queue {
    Node* front;
    Node* rear;
};

struct Stack {
    Node* top;
};

// Queue Functions
Queue* createQueue();
bool isEmpty(Queue* queue);
void enqueue(Queue* queue, coord item);
coord dequeue(Queue* queue);
coord peek(Queue* queue);

// Stack Functions
Stack* createStack();
bool isEmpty(Stack* stack);
void push(Stack* stack, coord item);
coord top(Stack* stack);
coord pop(Stack* stack);

// Maze Functions
bool isValid(int x, int y);
void init_arr(std::vector<std::vector<int>>& arr, int row, int col);
void move_fwd();
void turn_left();
void turn_right();
bool iswall_left();
bool iswall_right();
bool iswall_front();
void check_and_fill(std::vector<std::vector<int>>& arr, int row, int col, int value);
void init_flood(std::vector<std::vector<int>>& arr, int row, int col);
void init_flood_start(std::vector<std::vector<int>>& arr, int row_, int col_, int back_);
bool check_wall_angle(cell_info cell, int& dir);
cell_info cell_direction_adjust(cell_info cell);
void go_to_cell(int& angle_now, int dir);
coord get_min_neighbour(cell_info cell_wall, coord cur, std::vector<std::vector<int>>& arr, bool change_ = 0);
void flood(Stack* stack_flood, std::vector<std::vector<int>>& arr);
cell_info update_walls(int angle_now, int row, int col);
coord floodfill(coord start, coord dest, std::vector<std::vector<int>>& arr, int& angle_now);
void init_maze();

extern Queue* myQueue;

#endif //MAIN_H
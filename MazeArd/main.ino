#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Sensor_Data.h"
#include "Movement_Lib.h"
#include "motor_PID.h"

// Function Declarations
void initializeGPIO();
void initializeTimers();
void led();
void errorHandler();
void start();




//#include <Arduino.h>
#include <ArduinoSTL.h>
//#include <iostream>
//#include <string.h>
#include <Vector.h>
//#include <QueueArray.h> //writing custom structs and functions for queue
//#include <Stack.h> //also writing custom structs and functions for this

using namespace std;
#define UP 0
#define	DOWN 1
#define	LEFT 2
#define	RIGHT 3
const  int rows=16;
const  int cols=16;
typedef struct coor{
    int row;
    int col;
    int value;
}coord;
typedef struct cell_infos{
	// variables for north,east,south,west walls
	bool walls[4];
	bool visited;
  int angle_update; //Check functionality in dry run
  bool dead=0;
}cell_info;
typedef struct wall_mazes{
	cell_info cells[16][16];
}wall_maze;
wall_maze maze;
const int dx[] = {1, -1, 0, 0};
const int dy[] = {0, 0, -1, 1};

//DOING SHIT FOR QUEUES
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



// put function declarations here:
//int myFunction(int, int);

//Functions declarations for Queue
Queue* createQueue();
bool isEmpty(Queue* queue);
void enqueue(Queue* queue, coord item);
coord dequeue(Queue* queue);
coord peek(Queue* queue);

Stack* createStack();
bool isEmpty(Stack* stack);
void push(Stack* stack, coord item);
coord top(Stack* stack);
coord pop(Stack* stack);


bool isValid(int x, int y);
void init_arr(std::vector<std::vector<int>> &arr,int row,int col);
void move_fwd();
void turn_left();
void turn_right();
bool iswall_left();
bool iswall_right();
bool iswall_front();
void check_and_fill(std::vector<std::vector<int>> &arr,int row,int col,int value);
void init_flood(std::vector<std::vector<int>> &arr,int row,int col);
void init_flood_start(std::vector<std::vector<int>> &arr,int row_,int col_,int back_);
bool check_wall_angle(cell_info cell,int &dir);
cell_info cell_direction_adjust(cell_info cell);
void go_to_cell(int &angle_now,int dir);
coord get_min_neighbour(cell_info cell_wall,coord cur,std::vector<std::vector<int>> &arr,bool change_=0);
void flood(Stack* stack_flood,std::vector<std::vector<int>> &arr);
cell_info update_walls(int angle_now,int row,int col);
coord floodfill(coord start,coord dest,std::vector<std::vector<int>> &arr,int &angle_now);
void init_maze();

Queue* myQueue=createQueue(); //std::queue<coord> myQueue;

void setup() {
  // put your setup code here, to run once: 
     RobotControl robot;

     robot.initialize();
     //robot.run();

    
    //  setMotorSpeed(0,200);
    //  Serial.println("Motor0");
    //  setMotorSpeed(1,200);
    //  Serial.println("Motor1");
    setMotorSpeed1(100);
    Serial.println("Giving initial pwm");



     std::vector<std::vector<int>>arr;
     init_arr(arr,16,16); //gives -1 values to all cells
     init_flood(arr,7,7); //initializes flood (Basic Manhattan Distance)
     init_maze(); //initiaalizes the maze with 0 walls
     Serial.println("Initializing done!");
     // test_maze()
     coord start={0,0,arr[0][0]};
     coord dest={7,7,arr[7][7]};
     //update_wall_debug(arr);
     int angle_now=90;   //setting bot direction to NORTH
     coord new_coord;
     for(int m =0;m<3;m++)   //Number of iterations
     {
         new_coord=floodfill(start,dest,arr,angle_now);      // FLOODFILL  THIS COMPLETES FLOODFILL ONE WAY AND RETURNS p_return
         init_flood_start(arr,0,0,1);        //FLOODFILL BACKWARDS INITIALIZATION
         // update_wall_debug(arr);
         //std::cerr<<"done2"<<std::endl;
         new_coord=floodfill(new_coord,start,arr,angle_now);     //IMPLEMENT FLOODFILL BACKWARDS
         init_flood_start(arr,7,7,2);    //INVERT AAGAIUN
         // update_wall_debug(arr);
     }
  //int result = myFunction(2, 3);
} //Currently this is the code with backtracking 21/08/24 (9:50pm)

void loop() {


   // robot.initialize();

}

// put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }
bool isValid(int x, int y) {
    return (x >= 0 && x < rows && y >= 0 && y < cols);
}
void init_arr(std::vector<std::vector<int>> &arr,int row,int col)
{
    for(int i = 0 ;i<row;i++)
    {
        std::vector<int> arr_row;
        for(int j = 0 ;j<col;j++)
        {
            arr_row.push_back(-1);
        }
        arr.push_back(arr_row);
    }
}

/*
void move_fwd()
{}
void turn_left()
{}
void turn_right()
{}
bool iswall_left()
{return false;}
bool iswall_right()
{return false;}
bool iswall_front()
{return false;}
*/

//QUEUE KE LIYE FUNCTIONS
Queue* createQueue() {
  Queue* queue = new Queue();
  queue->front = queue->rear = NULL;
  return queue;
}
bool isEmpty(Queue* queue) {
  return queue->front == NULL;
}
void enqueue(Queue* queue, coord item) {
  Node* newNode = new Node();
  newNode->data = item;
  newNode->next = NULL;

  if (isEmpty(queue)) {
    queue->front = queue->rear = newNode;
  } else {
    queue->rear->next = newNode;
    queue->rear = newNode;

  }
}
coord dequeue(Queue* queue) {
  if (isEmpty(queue)) {
    // Handle error: queue is empty
    // You can return a default value or throw an exception
    coord emptyData;
    return emptyData;
  }

  Node* temp = queue->front;
  coord item = temp->data;
  queue->front = temp->next;

  if (queue->front == NULL) {
    queue->rear = NULL;
  }

  delete temp;
  return item;
}
coord peek(Queue* queue) {
  if (isEmpty(queue)) {
    // Handle error: queue is empty
    // You can return a default value or throw an exception
    coord emptyData;
    return emptyData;
  }
  return queue->front->data;
}

Stack* createStack() {
  Stack* stack = new Stack();
  stack->top = NULL;
  return stack;
}
bool isEmpty(Stack* stack) { //method overloading
  return stack->top == NULL;
}
void push(Stack* stack, coord item) {
  Node* newNode = new Node();
  newNode->data = item;
  newNode->next = stack->top;
  stack->top = newNode;
}
coord top(Stack* stack) {
  if (isEmpty(stack)) {
    // Handle error: stack is empty
    // You can return a default value or throw an exception
    coord emptyData;
    return emptyData;
  }
  return stack->top->data;
}
coord pop(Stack* stack) {
  if (isEmpty(stack)) {
    // Handle error: stack is empty
    // You can return a default value or throw an exception
    coord emptyData;
    return emptyData;
  }
  Node* temp = stack->top;
  coord item = temp->data;
  stack->top = temp->next;
  delete temp;
  return item;
}

void check_and_fill(std::vector<std::vector<int>> &arr,int row,int col,int value)
{
    if(row<0 ||col<0||row>=arr.size()||col>=arr.size()||arr[row][col]!=-1)return;
    value+=1;
    coord point={row,col,value};
    enqueue(myQueue, point);//myQueue.push(point);
    arr[row][col]=value;
}
void init_flood(std::vector<std::vector<int>> &arr,int row,int col)
{
   
    int count_=0;
    coord point={row,col,count_};
    enqueue(myQueue, point);//myQueue.push(point);
    arr[row][col]=0;
    coord point2={row+1,col,count_};
    enqueue(myQueue, point2);//myQueue.push(point2);
    arr[row+1][col]=0;
    coord point3={row,col+1,count_};
    enqueue(myQueue, point3);//myQueue.push(point3);
    arr[row][col+1]=0;
    coord point4={row+1,col+1,count_};
    enqueue(myQueue, point4);//myQueue.push(point4);
    arr[row+1][col+1]=0;
    while (!isEmpty(myQueue)) {
        //coord frontCoord = myQueue.front(); 
        //myQueue.pop();
        coord frontCoord = dequeue(myQueue); 
        check_and_fill(arr,frontCoord.row+1,frontCoord.col,frontCoord.value);
        check_and_fill(arr,frontCoord.row-1,frontCoord.col,frontCoord.value);
        check_and_fill(arr,frontCoord.row,frontCoord.col+1,frontCoord.value);
        check_and_fill(arr,frontCoord.row,frontCoord.col-1,frontCoord.value);
        for(int i=0;i<16;i++)
        {
            for(int j = 0 ;j<16;j++)
            {
                //API::setText(i,j,"0");      // Checking if API works and when does init_floodfill occur
            }                               // Ok so init_floodfill occurs for like a split second in the very beginning.
        }
    
    }
    
    
}
void init_flood_start(std::vector<std::vector<int>> &arr,int row_,int col_,int back_)
{
    
    int count_=0;
    for(int i=0;i<16;i++)
    {
        for(int j = 0 ;j<16;j++)
        {
            arr[i][j]=-1;
            if(back_==2&&maze.cells[i][j].visited==false){
                arr[i][j]=255;
                maze.cells[i][j].dead=true;
            }
           
        }
        //  if(back_==2)std::cerr<<""<<std::endl;
    }
    if(back_!=1) //ignored when backtracking
    {
        coord point2={row_+1,col_,count_};
        enqueue(myQueue, point2);//myQueue.push(point2);
        arr[row_+1][col_]=0;
        coord point3={row_,col_+1,count_};
        enqueue(myQueue, point3);//myQueue.push(point3);
        arr[row_][col_+1]=0;
        coord point4={row_+1,col_+1,count_};
        enqueue(myQueue, point4);//myQueue.push(point4);
        arr[row_+1][col_+1]=0;
    }//seete bajake bol bhaiya all izz well
    coord point={row_,col_,count_};
    enqueue(myQueue, point);//myQueue.push(point);
    arr[row_][col_]=0;
    while(isEmpty(myQueue))
    {
        //coord frontCoord = myQueue.front(); 
        //myQueue.pop();
        coord frontCoord = dequeue(myQueue); 
          for (int i = 0; i < 4; ++i) {
                int newRow = frontCoord.row + dy[i]; // 0 0 -1 1
                int newCol = frontCoord.col + dx[i]; // 1 -1 0 0
                // std::cerr <<dir<< new_cell.walls[dir]<<std::endl ;
                // bool check_=check_wall_angle(maze.cells[cur_stack.row][cur_stack.col],ind_);
                bool check_=maze.cells[frontCoord.row][frontCoord.col].walls[i];
                if(!check_)check_and_fill(arr,newRow,newCol,frontCoord.value);
          }
        //   std::cerr<<"size:"<<myQueue.size()<<std::endl;
        //   if(myQueue.size()>120){
        //     log("fulllll");
        //     break;
        //   }
    } 
}
bool check_wall_angle(cell_info cell,int &dir)
{
    switch(cell.angle_update)
    {
        case 90:
            break;
        case 270:
            if(dir%2==0)dir+=1;
            else dir-=1;
            break;
        case 0:
            if(dir==0 || dir ==1)dir+=2;
            else if(dir==2)dir=1;
            else dir=0;
            break;
        case 180:
             if(dir==2 || dir ==3)dir-=2;
            else if(dir==0)dir=3;
            else dir=2;
            break;
    }
    return cell.walls[dir];
}
cell_info cell_direction_adjust(cell_info cell)     //ADJUSTS ORIENTATION
{
    //  std::cerr<<"A:"<<cell.angle_update<<"-L:";
    cell_info cell_new;
    cell_new=cell;
    for(int i=0;i<4;i++)
    {
        int ind = i;
    // std::cerr<<cell.walls[i];
    // std::cerr<<i<<"->";
    switch(cell.angle_update)
        {
            case 90:
                break;
            case 270:
                if(i%2==0)ind+=1;
                else ind-=1;
                break;
            case 0:
                if(i==0 || i ==1)ind+=2;
                else if(i==2)ind=1;
                else ind=0;
                break;
            case 180:
                if(i==2 || i ==3)ind-=2;
                else if(i==0)ind=3;
                else ind=2;
                break;
        }
        //  std::cerr<<ind<<"|";
        cell_new.walls[i]=cell.walls[ind];
    }
    return cell_new;
}
void go_to_cell(int &angle_now,int dir)
{
  Serial.println("Inside go_to_cell");
    switch(dir)
            {
                case -1:
                    //log("not dir");
                    Serial.println("-1 condition");
                    break;
                case UP:
                    // log("forward");
                    Serial.print("Move_fwd...");
                    move_fwd();
                    break;
                case DOWN:
                    // log("Down");
                    Serial.println("Down");
                    angle_now-=180;
                     turn_right();
                     turn_right();
                     move_fwd();
                      break;
                case LEFT:
                    // log("Left");
                    Serial.println("Left");
                    angle_now+=90;
                    Serial.print("Left...");
                    turn_left();
                    move_fwd();
                    break;
                case RIGHT:
                    // log("right");
                    Serial.println("Right");
                    angle_now-=90;
                    Serial.print("Right...");
                    turn_right();
                    move_fwd();
                    break;
                default:
                    Serial.println("No direction called");
                    break;
            }
            angle_now = angle_now % 360;

            if (angle_now < 0) {
                angle_now += 360;
            }
}
coord get_min_neighbour(cell_info cell_wall,coord cur,std::vector<std::vector<int>> &arr,bool change_=0)
{
    int min_neightbor=255;     //neightbor hehe API::wall
    coord next_step;
    next_step.value=-1;
    int ind;
     for (int dir = 0; dir < 4; ++dir) {
                int newRow = cur.row + dy[dir]; // 0 0 -1 1
                int newCol = cur.col + dx[dir]; //1 -1 0 0
                ind=dir;
                bool check_=cell_wall.walls[dir];
                if(change_)check_=check_wall_angle(cell_wall,ind);
                // std::cerr << check_;
                if(isValid(newRow,newCol) && !check_)
                {
                    if(arr[newRow][newCol]<=min_neightbor)
                    {
                        min_neightbor=arr[newRow][newCol];
                        next_step.row=newRow;
                        next_step.col=newCol;
                        next_step.value=ind;
                    }
                }
            }
    return next_step;
}
void flood(Stack* stack_flood ,std::vector<std::vector<int>> &arr)
{
    coord cur_stack;
    coord next_step;
     while(!isEmpty(stack_flood))
        {
            // cur_stack=stack_flood.top();
            // stack_flood.pop(); 
            cur_stack=pop(stack_flood);
            int min_neightbor=255;
            bool check_;
            next_step=get_min_neighbour(maze.cells[cur_stack.row][cur_stack.col],cur_stack,arr);
            min_neightbor=arr[next_step.row][next_step.col];
            if(arr[cur_stack.row][cur_stack.col]-1 != min_neightbor )   //Here is where that check for if we need to recalc manhattan distances happens
            {
                for(int i =0 ;i<4;i++)
                {
                    coord cur_add;
                    cur_add.row= cur_stack.row + dy[i]; // 0 0 -1 1
                    cur_add.col= cur_stack.col + dx[i]; //1 -1 0 0 
                    check_=maze.cells[cur_stack.row][cur_stack.col].walls[i];
                    if(isValid(cur_add.row,cur_add.col) &&arr[cur_add.row][cur_add.col]!=0&&!check_)    //Basically 
                    {
                        push(stack_flood, cur_add);//stack_flood.push(cur_add);
                    }
                }
                if(arr[cur_stack.row][cur_stack.col]!=0)
                arr[cur_stack.row][cur_stack.col]=min_neightbor+1;  //This enters the updated floodfill manhattan distance
                // update_wall_debug(arr);
                // log("added");
            }
            //int stack_size=stack_flood.size();
            //if(stack_size>=35){
            //    log("full stack");
             //   for(int i=0;i<stack_size;i++)
              //  {
            //        stack_flood.pop();
              //  }
              //  return;
            //}
        }
}
cell_info update_walls(int angle_now,int row,int col)
{
    cell_info new_cell;
    new_cell.angle_update=angle_now;
    new_cell.walls[UP]=iswall_front();
    new_cell.walls[DOWN]=0;
    new_cell.walls[LEFT]=iswall_left();
    new_cell.walls[RIGHT]=iswall_right();
    new_cell.dead=0;
    new_cell.visited=1;
    maze.cells[row][col]=cell_direction_adjust(new_cell);   //ADJUSTS THE WALL DATA DEPENDING ON ORIENTATION
    if(new_cell.walls[UP]==1&&new_cell.walls[LEFT]==1&&new_cell.walls[RIGHT]==1&&row!=0&&col!=0)
    {
        //log("dead");
        maze.cells[row][col].dead=1;
    }
    for(int i=0;i<4;i++)    //Iterate through directions and updates the wall data for adjacent walls
    {
        int newRow=row+dy[i];
        int newCol=col+dx[i];
        if(isValid(newRow,newCol))  ///checks validity of cell
        {
            if(i==UP)maze.cells[newRow][newCol].walls[DOWN]=maze.cells[row][col].walls[UP];
            else if(i==LEFT)maze.cells[newRow][newCol].walls[RIGHT]=maze.cells[row][col].walls[LEFT];       //Updating wall data for adjacent cell
            else if(i==RIGHT)maze.cells[newRow][newCol].walls[LEFT]=maze.cells[row][col].walls[RIGHT];
        }
    }
    return new_cell;
}
coord floodfill(coord start,coord dest,std::vector<std::vector<int>> &arr,int &angle_now)
{
    Queue* path_queue=createQueue();//std::queue<coord>path_queue;    //Initalising the queue of coords that will store our path of cells
    enqueue(path_queue, start);//path_queue.push(start);
    coord cur=start;
    cell_info new_cell; //Stores cell data for new cell
    // cell_info new_cell;
    Stack* stack_flood=createStack();//std::stack<coord>stack_flood;   //Initializing a stack of coords as well for ................... ?
    push(stack_flood,start);//stack_flood.push(start);
    int path_distance_value_find=0; //Counter to store path travelled in the iteration
    int save_row,save_col;
    coord next_step;
    Serial.println("Inside floodfill!");
    while(1)
    {
      Serial.println("Inside While\n");
        if(!isEmpty(path_queue)) 
        {
          
            cur=peek(path_queue);//cur=path_queue.front(); //Cur is the most recent cell in path queue
            new_cell=update_walls(angle_now,cur.row,cur.col);   //Updates the data of the walls around the current cell and updates the data for the adjacent cells based on what we know for the current cell 
            if(arr[cur.row][cur.col]==arr[dest.row][dest.col])break;    //Checks if we have reached the destination
            flood(stack_flood,arr); //FLOODS THE MATRIX ALONG WITH UPDATIUNG THE MANHATTAN DISTANCES
            dequeue(path_queue);//path_queue.pop(); 
            next_step=get_min_neighbour(new_cell,cur,arr,1);    //Get next cell to go to
            enqueue(path_queue, next_step);//path_queue.push(next_step);
            push(stack_flood,next_step);//stack_flood.push(next_step);        //Updating the stack and queue
            Serial.println("go_to_cell\n");
            go_to_cell(angle_now,next_step.value);      //Move to min neighbour   
            path_distance_value_find++;     //Counter to update the path distance travelled
        }
        else{
            //log("empty Queue- break");
            break;
        }
        // std::cerr<<"cur:"<<cur.value<<"-dest:"<<dest.value<<std::endl;
    }
    while(!isEmpty(path_queue)) dequeue(path_queue); 
    // new_cell=update_walls(angle_now,cur.row,cur.col);
    //END OF FLOODFILL GOING
    //std::cerr<<"total_cost:"<<path_distance_value_find<<std::endl;
    coord p_return={next_step.row,next_step.col,0};         //p_return mystery
    return p_return;
}
void init_maze()
{
    for(int i =0;i<16;i++)
    {
        for(int j=0;j<16;j++)
        {
            maze.cells[i][j].visited=0;
            maze.cells[i][j].angle_update=90;
            maze.cells[i][j].dead=0;
            for(int k = 0 ;k<4;k++)maze.cells[i][j].walls[k]=0;
        }
    }
}
//Sensor_Data part ends here

void initializeGPIO() {
    pinMode(13, OUTPUT);  // Configuring pin 13 as output
}

void initializeTimers() {
    // For basic timer functionality, Arduino uses millis() or micros().
    // PWM on pins can be set using analogWrite().
    // If advanced timing is needed, consider using timer libraries like TimerOne or TimerThree.
}

void led() {
    // Implement your LED functionality here.
}



void errorHandler() {
    // Implement your error handling here.
}


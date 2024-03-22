/* 
███████╗██╗███╗░░██╗░█████╗░██╗░░░░░
██╔════╝██║████╗░██║██╔══██╗██║░░░░░
█████╗░░██║██╔██╗██║███████║██║░░░░░
██╔══╝░░██║██║╚████║██╔══██║██║░░░░░
██║░░░░░██║██║░╚███║██║░░██║███████╗
╚═╝░░░░░╚═╝╚═╝░░╚══╝╚═╝░░╚═╝╚══════╝

╔═╗╔═╦═╗╔═╦═══╗
║║╚╝║║║╚╝║║╔═╗║
║╔╗╔╗║╔╗╔╗║╚══╗
║║║║║║║║║║╠══╗║
║║║║║║║║║║║╚═╝║
╚╝╚╝╚╩╝╚╝╚╩═══╝
 *  Programme for 1st Chip 
 * version 2.0
 * 24.09.2022   19.57
 * 
 * With Sharp sharp sensor 
 * PID control
 * Turn according to sharp IR
 * Stop if front detected
 * 
 * Analog pin 5 -front sensor
 * Analog pin 4 -left sensor
 * analog pin 3 -right sensor
 */

#include <QueueList.h>
#include <SharpDistSensor.h>

#define UNDEFINED -10
#define LENGTH 14
#define AREA 196
#define AREA_WALLS 14 * (14 - 1) - 1

int lastCol = 0;
int lastRow = 0;
int curCol = 0;
int curRow = 0;
int corner = UNDEFINED;
int floodfillArr[AREA] = {};

int turns = 0;

int senserVal[3] = {0,0,0};  // 0-front, 1-Left, 2-Right

int threshold = 100;
int direction = 0;
int lastAction = 0;

float kp=150;
float kd=10;
float ki=2;

int lstval=0;
long total_err=0;
int M=250;

boolean wallsHorizontal[AREA_WALLS];
boolean wallsVertical[AREA_WALLS] = { true };
static boolean enabled = false;

// Threshold values
#define FRONT_DISABLE_THRESHOLD 800
#define FRONT_WALL_THRESHOLD 270
#define STOP_THRESHOLD 930
#define SIDE_WALL_THRESHOLD 370
#define SIDE_OPENING_THRESHOLD 150
 
#define RF 5
#define RB 4
#define LF 3
#define LB 2


// Analog pin to which the sensor is connected
#define NUM_READS 50
#define FRONT A3
#define FRONT_LEFT A2
#define FRONT_RIGHT A3

//WANT TO REMOVE THIS 
const byte sensorPin0 = A3;
const byte sensorPin1 = A2;
const byte sensorPin2 = A4;

// Window size of the median filter (odd number, 1 = no filtering)
const byte medianFilterWindowSize = 5;

// Create an object instance of the SharpDistSensor class
SharpDistSensor sensor0(sensorPin0, medianFilterWindowSize);
SharpDistSensor sensor1(sensorPin1, medianFilterWindowSize);
SharpDistSensor sensor2(sensorPin2, medianFilterWindowSize);



void setup() {

  initializeFloodfill();
  floodfill();
for(int i=2;i<6;i++){
  pinMode(i,OUTPUT);
}
  // Sense  walls in north straight away
  boolean north, east, south, west;
  sense(north, east, south, west);
Serial.begin(9600);

}



void loop() {
  //initailize 
  senseIR();
  int z = getFloodfillValue(6,7);
  Serial.println(z);
  
//Serial.print(senserVal[0]);Serial.print(" ");
//Serial.print(senserVal[1]);Serial.print("  ");
//Serial.println(senserVal[2]);

//check if reach to center
if(getFloodfillValue(curRow, curCol) == 0) {
    enabled = false; return;
  }
  
int nextVal = AREA, nextRow, nextCol;
boolean north, east, south, west; 


// Look al all 4 sides to the current cell and determine the lowest floodfill value...
for(int dir = 0; dir < 4; dir++) {
  boolean hasWall = wallExists(curRow, curCol, dir);

  int row = curRow, col = curCol;
  getAdjacentWall(row, col, dir);

  int val = getFloodfillValue(row, col);

  if(!hasWall && (val == nextVal && dir == direction || val < nextVal)) {
    nextRow = row;
    nextCol = col;
    nextVal = val;
  }
}

  
// If we just rotated... sense first to update floodfill
  if(lastAction == 2 || corner == UNDEFINED) {
    sense(north, east, south, west);
  }
  
  if(corner == UNDEFINED) {
    nextRow = curRow + 1; nextCol = curCol;
    moveTo(nextRow, nextCol);
    return;
  }
int newDirection = curCol == nextCol ? ( curRow < nextRow ? 0 : 2 ) : 
    corner == 3 ? ( curCol < nextCol ? 1 : 3 ) : ( curCol < nextCol ? 3 : 1 );

  // If we need to rotate... go!
  if(newDirection != direction) {

    // Keep track of the number of turns and set direction
    turns -= direction - newDirection;
    direction = newDirection;
  }
  
  // No rotation? Sense and move
  else {
    // Sense the forward blocks surrounding area using sensors
    sense(north, east, south, west);

    // Check for a dead end (but make sure it isn't the center!)
    if(getFloodfillValue(nextRow, nextCol) < 1 || (north + east + south + west) < 3) {
      moveTo(nextRow, nextCol);
    }
  }



//Amala pid code function  
//pidSense();
}

/*

╔═══╦══╦═══╗╔═══╦═══╦═╗─╔╦════╦═══╦═══╦╗──╔╗──╔══╦═╗─╔╦═══╗
║╔═╗╠╣╠╩╗╔╗║║╔═╗║╔═╗║║╚╗║║╔╗╔╗║╔═╗║╔═╗║║──║║──╚╣╠╣║╚╗║║╔═╗║
║╚═╝║║║─║║║║║║─╚╣║─║║╔╗╚╝╠╝║║╚╣╚═╝║║─║║║──║║───║║║╔╗╚╝║║─╚╝
║╔══╝║║─║║║║║║─╔╣║─║║║╚╗║║─║║─║╔╗╔╣║─║║║─╔╣║─╔╗║║║║╚╗║║║╔═╗
║║──╔╣╠╦╝╚╝║║╚═╝║╚═╝║║─║║║─║║─║║║╚╣╚═╝║╚═╝║╚═╝╠╣╠╣║─║║║╚╩═║
╚╝──╚══╩═══╝╚═══╩═══╩╝─╚═╝─╚╝─╚╝╚═╩═══╩═══╩═══╩══╩╝─╚═╩═══╝

*/

void pidSense(){
  if(senserVal[0]>100){
  int err=senserVal[2]-senserVal[1];
  int PIDval=PIDcontroll(err);

    TURN(M+PIDval,M-PIDval);}
   
  else{
  STOP();
    
}
  
  }

int PIDcontroll(int err){
   int Pterm = err*kp;
  int Dterm = (err-lstval)*kd;
  int Iterm = total_err*ki;
  
  int diff=Pterm;

  lstval=err;
  total_err =+ err;

  return diff;
}

void TURN(int LSpeed, int RSpeed){
  if(LSpeed>255){
    LSpeed=255;
  }
  if(LSpeed<=0){
    LSpeed=0;
  }
  if(RSpeed>255){
    RSpeed=255;
  }
  if(RSpeed<=0){
    RSpeed=0;
  }
  analogWrite(LF,LSpeed);
  digitalWrite(LB,LOW);
  analogWrite(RF,RSpeed);
  digitalWrite(RB,LOW);
 
}


/*
╔═══╦╗──╔═══╦═══╦═══╗╔═══╦══╦╗──╔╗
║╔══╣║──║╔═╗║╔═╗╠╗╔╗║║╔══╩╣╠╣║──║║
║╚══╣║──║║─║║║─║║║║║║║╚══╗║║║║──║║
║╔══╣║─╔╣║─║║║─║║║║║║║╔══╝║║║║─╔╣║─╔╗
║║──║╚═╝║╚═╝║╚═╝╠╝╚╝║║║──╔╣╠╣╚═╝║╚═╝║
╚╝──╚═══╩═══╩═══╩═══╝╚╝──╚══╩═══╩═══╝
*/

void initializeFloodfill() {
  for(int i = 0; i < AREA; i++) {
    floodfillArr[i] = UNDEFINED;
  }
}

 
void floodfill() {
  QueueList <int> queue;
  int i = 0;

  // Clean the array
  initializeFloodfill();
  
  // Push middle of maze onto queue
  queue.push( rowColtoZ( floor(LENGTH / 2), floor(LENGTH / 2) ) );
  
    
  while(!queue.isEmpty()) {
    int x = queue.pop();
    int z = x & 255;
    
    int row, col;
    zToRowCol( z, row, col );
    
    int val = (x >> 8) & 255;
    int curVal = floodfillArr[ z ];

    // Floodfill this cell
    if((curVal != UNDEFINED && curVal <= val) || z < 0 || z > AREA) { continue; }
    floodfillArr[ z ] = val;

    // Check the cell to the north
    if(!wallExists(row, col, 0)) {
      queue.push( rowColtoZ(row + 1, col) | ((val + 1) << 8) );
    }

    // Check the cel to the east
    if(!wallExists(row, col, 1)) {
      queue.push( rowColtoZ(row, col + 1) | ((val + 1) << 8) );
    }

    // Check the cell to the south
    if(!wallExists(row, col, 2)) {
      queue.push( rowColtoZ(row - 1, col) | ((val + 1) << 8) );
    }

    // Check the cell to the west
    if(!wallExists(row, col, 3)) {
      queue.push( rowColtoZ(row, col - 1) | ((val + 1) << 8) );
    }

    i++;
  }
  
  }


 int getFloodfillValue(int row, int col) {
 return floodfillArr[ rowColtoZ(row, col) ];
}




int rowColtoZ(int row, int col) {
  return abs(LENGTH * row + col);
}

int colRowtoZ(int row, int col) {
  return abs(LENGTH * col + row);
}

void zToRowCol(int z, int &row, int &col) {
  row = floor(z / LENGTH);
  col = z % LENGTH;
}

boolean wallExists(int row, int col, int dir) {
  if(corner == UNDEFINED) { return false; }

  int index = calculateWallIndex(row, col, dir);
  boolean *arr = dir == 0 || dir == 2 ? wallsHorizontal : wallsVertical;
  
  return (index < 0 || index > AREA_WALLS) ? true : arr[index];
}



void getAdjacentWall(int &row, int &col, int dir) {
  int subject = corner == 3 ? 1 : -1;
    
  switch(dir) {
    case 0: row = row + 1; break;
    case 1: col = col + subject; break;
    case 2: row = row - 1; break;
    case 3: col = col - subject; break;
  }
}

/*
╔═══╦═══╦═╗─╔╦═══╦═══╦═══╦═══╗
║╔═╗║╔══╣║╚╗║║╔═╗║╔═╗║╔═╗║╔═╗║
║╚══╣╚══╣╔╗╚╝║╚══╣║─║║╚═╝║╚══╗
╚══╗║╔══╣║╚╗║╠══╗║║─║║╔╗╔╩══╗║
║╚═╝║╚══╣║─║║║╚═╝║╚═╝║║║╚╣╚═╝║
╚═══╩═══╩╝─╚═╩═══╩═══╩╝╚═╩═══╝
*/

void senseIR(){
  senserVal[0] = sensor0.getDist();
  senserVal[1] = sensor1.getDist();
  senserVal[2] = sensor2.getDist();
}


void sense(boolean &north, boolean &east, boolean &south, boolean &west) {
  int row = curRow, col = curCol;
  getAdjacentWall(row, col, direction);
        
  boolean sensors[] = {
    senseWall(FRONT),
    senseWall(FRONT_LEFT),
    false,
    senseWall(FRONT_RIGHT)
  };

  // Determine absolute wall values
  int shift = turns % 4;
  if(turns > 0) { shift = 4 - shift; }
  north = sensors[ (0 + shift) % 4 ];
  east = sensors[ (1 + shift) % 4 ];
  south = sensors[ (2 + shift) % 4 ];
  west = sensors[ (3 + shift) % 4 ];

  if(corner == UNDEFINED && (!east || !west)) {
    corner = !east ? 3 : 1;
    // Store side walls for all cells we've visisted before corner detection
    for(int i = 0; i < curRow + 1; i++) {
      setWall(i, curCol, 1, true);
      setWall(i, curCol, 3, true);
    }
  }        
  // Floodfill
  floodfill();
}


boolean senseWall(int pin) {
  int value = senseValue(pin);
  
  if(pin == FRONT) {
    return value > FRONT_WALL_THRESHOLD; 
  }
  else {
    return value > SIDE_WALL_THRESHOLD;
  }
}

int senseValue(int pin) {
  // Ensure we don't read the front sensor if within a given range, as the readings are inaccurate
  // Instead, we take an average of the two side sensors
  //if(pin == FRONT && !use_front) {
  //  return (readSensorAnalog(FRONT_LEFT) + readSensorAnalog(FRONT_RIGHT)) / 2;
  //}
  
  return readSensorAnalog(pin);
  
  //if Avg side sensor greater than 850 stop!
}

int readSensorAnalog(int pin) {
  int sortedValues[NUM_READS];
  int i, j = 0, k;

  for(i = 0; i < NUM_READS; i++) {
    int value = analogRead(pin);

    if(value < sortedValues[0] || i == 0) {
      j = 0;
    }
    else {
      for(j = 1; j < i; j++) {
        if(sortedValues[j - 1] <= value && sortedValues[j] >= value ) {
          break;
        }
      }
    }

    for(k = i; k > j; k--) {
      sortedValues[k] = sortedValues[k - 1];
    }
    sortedValues[j] = value;
        }

  int returnVal = 0;
  for(i = NUM_READS / 2 - 5; i < (NUM_READS / 2 + 5); i++) {
    returnVal += sortedValues[i];
  }

  return int(returnVal / 10);
}


/*

╔═╗╔═╦═══╦════╦═══╦═══╦═══╗
║║╚╝║║╔═╗║╔╗╔╗║╔═╗║╔═╗║╔═╗║
║╔╗╔╗║║─║╠╝║║╚╣║─║║╚═╝║╚══╗
║║║║║║║─║║─║║─║║─║║╔╗╔╩══╗║
║║║║║║╚═╝║─║║─║╚═╝║║║╚╣╚═╝║
╚╝╚╝╚╩═══╝─╚╝─╚═══╩╝╚═╩═══╝

*/

void moveTo(int row, int col) {
  int x_move = col - lastCol;
  int y_move = row - lastRow;

  if(x_move > 0){
    TURN90RIGHT();
    forward_one_cell();
    }
   else if(x_move < 0){
    TURN90LEFT();
    forward_one_cell();
      
    }
   else if (y_move>0){
    forward_one_cell();
      
      }
   else{
    TURN90RIGHT();
      
      }
   
  // Store the current values and set new pos
  lastCol = curCol;
  lastRow = curRow;
  curCol = col;
  curRow = row;
  //create a move function
  

}

void forward_one_cell(){
  int startdistance = getencoderVal();
  int distance = startdistance - getencoderVal();
  while(distance < 16){
    FORWARD();
    distance = startdistance - getencoderVal();
    }
   Serial.write(0);
  }

int getencoderVal(){
  Serial.write(1);
  if(Serial.available()){
    int distance = Serial.read();
     return distance;
    }
 
  }

void TURN90RIGHT(){
  
  int startAngle = getgyroVal();
  int angle = startAngle - getgyroVal();
  while(angle < 90){
    ROTATERIGHT();
     angle = startAngle - getgyroVal();
    }
  }


void TURN90LEFT(){
  
  int startAngle = getgyroVal();
  int angle = startAngle - getgyroVal();
  while(angle > -90){
    ROTATELEFT();
     angle = startAngle - getgyroVal();
    }
  }

int getgyroVal(){
  Serial.write(2);
  if(Serial.available()){
    int angle = Serial.read();
  return angle;
  }
  
  
  }





void FORWARD(){
  digitalWrite(RF,HIGH);
  digitalWrite(RB,LOW);
  digitalWrite(LF,HIGH);
  digitalWrite(LB,LOW);
}

void STOP(){
  digitalWrite(RF,LOW);
  digitalWrite(RB,LOW);
  digitalWrite(LF,LOW);
  digitalWrite(LB,LOW);
}

void TURNRIGHT(){
  digitalWrite(RF,LOW);
  digitalWrite(RB,LOW);
  digitalWrite(LF,HIGH);
  digitalWrite(LB,LOW);
}

void TURNLEFT(){
  digitalWrite(RF,HIGH);
  digitalWrite(RB,LOW);
  digitalWrite(LF,LOW);
  digitalWrite(LB,LOW);
}

void ROTATELEFT(){
  digitalWrite(RF,HIGH);
  digitalWrite(RB,LOW);
  digitalWrite(LF,LOW);
  digitalWrite(LB,HIGH);
}

void ROTATERIGHT(){
  digitalWrite(RF,LOW);
  digitalWrite(RB,HIGH);
  digitalWrite(LF,HIGH);
  digitalWrite(LB,LOW);
}


/*

╔═╗╔═╦═══╦═╗╔═╦═══╦═══╦╗──╔╗
║║╚╝║║╔══╣║╚╝║║╔═╗║╔═╗║╚╗╔╝║
║╔╗╔╗║╚══╣╔╗╔╗║║─║║╚═╝╠╗╚╝╔╝
║║║║║║╔══╣║║║║║║─║║╔╗╔╝╚╗╔╝
║║║║║║╚══╣║║║║║╚═╝║║║╚╗─║║
╚╝╚╝╚╩═══╩╝╚╝╚╩═══╩╝╚═╝─╚╝

*/


void setWall(int row, int col, int dir, boolean state) {
  if(corner == UNDEFINED) { return; }

  int index = calculateWallIndex(row, col, dir);
  boolean *arr = dir == 0 || dir == 2 ? wallsHorizontal : wallsVertical;
  
  // Ensure we don't go outside the walls range
  if(index < 0 || index > AREA_WALLS) { return; }

  // Set the value
  arr[index] = state;
}

int calculateWallIndex(int row, int col, int dir) {
  int index;

  // Up or down
  if(dir == 0 || dir == 2) {
    index = rowColtoZ(
      dir == 0 ? row : row - 1, col
    );
  }

  // Right or left
  else if(dir == 1 || dir == 3) {
    index = colRowtoZ(
      corner == 3 ? 
        (dir == 1 ? col : col - 1) : 
        (dir == 3 ? col : col - 1),
      row
    );
  }

  return index;
}

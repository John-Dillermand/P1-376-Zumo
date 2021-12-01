#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>
#include <Zumo32U4IMU.h>
#include <Arduino.h>
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LCD lcd;
Zumo32U4IMU imu;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;

double x = 0, y = 0, pose = 0, temp, deg = 0; //the x coordinate, the y coordinate, the position angle compared to the x-axis given in radians, "deg" For cenverting radians to degrees 
//Distance on the right and left will always come from an equal amount of time
double distRight, distLeft; //the dist traveled on the right wheel, dist traveled on left wheel, counts from 
int Position = 0; // The angle at which the robot is pointing in the global coordinate system

double poseVector[3] = {0,0,0};  // The "beginning" vector 
double newPose[3]; //A "new position" vector for new coordinates and angles from the distance traveled given that the starting position is [0,0,0]
int counts[2]; //an array for the numbers retrieved from the encoders
int rememberCounts[2] = {0,0}; //a vector to remember the counts 

int i = 0; //Iterations of the for-loop
int maxY = 100; //The max y-distance on the field
int wildOats[2][5] = {{0,10,30,60,90},{0,30,90,10,60}}; //An array describing the positions of the wild oats

#define NUM_SENSORS 5 //Number of activated sensors
uint16_t lineSensorValues[NUM_SENSORS]; //Some array that contains the raw read values from the sensors between 0-2000
bool useEmitters = true;
struct LineSensorsWhite { //Datatype that stores the boolean values for the sensorStates
  bool L;
  bool LC;
  bool C;
  bool RC;
  bool R;
};

bool farLeft = false;
bool left = false;
bool Center = false;
bool right  = false;
bool farRight = false;

LineSensorsWhite sensorsState = {0, 0, 0, 0, 0};

int threshold[NUM_SENSORS] = {800, 400, 400, 400, 800};

double distToPoint[5] = {0, 0, 0, 0, 0}; //This array storages the distance, that describe the accuracy the zumo hits the point with
int j = 0; //This is the variable used to update the elements in distToPoint[] in the same order as the zumo hits the point





void returnYmax(){ //Drives the distance needed to go from current y-position to the max y-value on the field 
  driveDist(maxY - poseVector[1]);
  resetCounts();//Reseting the encoder counts as to not confuse the "location" function
}

void returnTo0(){ //Driving the distance needed to go from the current y-position to the x-axis
  driveDist(-(0-wildOats[1][i+1]));
  resetCounts(); //Reseting the encoder counts as to not confuse the "driveDist" function, didn't work when I put it in the function
}

void continuing(){
  driveDist(wildOats[0][i+1]-wildOats[0][i]); //Drives to the x-coordinate of the wild oat (Maybe change this to (wildOats[0][i+1]-poseVector[0]))
  resetCounts();
  delay(100);
  //If-statement that determines wether the robot should turn right or left before driving on the y-axis
  if (poseVector[1] > (maxY-2) && poseVector[1] < (maxY+2)){
    turnRight();
  }
  else {
    turnLeft();
  }
}

void driveToSpot(){ //Function driving the robot to the y-value of the wild oat
  if(poseVector[1] <(maxY+2) && poseVector[1] > (maxY-2)){ //Determines same thing as if-statement in "continuing" (should maybe use a bool instead) 
    driveDist(maxY-wildOats[1][i+1]); //Then drives to the wild oats y-value
  }
  else{
    driveDist(wildOats[1][i+1]);
    }
    resetCounts();
    delay(100);
    turnStraight(); //This will be used when the robot has to drive out in the field
}

void returning(){ //A function that returns the robot to either the x-axis or the max y-value of the field
  //depending on wether it is closer to the max y-value or the x-axis
  double xRoute = wildOats[1][i] + wildOats[1][i+1];
  double yRoute = (maxY-wildOats[1][i]) +(maxY-wildOats[1][i+1]);
  if (xRoute >= yRoute){
    turnLeft();
    returnYmax();
    turnStraight();
  }
  else {
   turnRight();
   returnTo0();
   turnStraight();
  }
}


void readSensors(LineSensorsWhite &state) {
  // Next line reads the sensor values and store them in the array lineSensorValues , aparameter passed by reference
  lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
  // In the following lines use the values of the sensors to update the struct
  sensorsState = {false, false, false, false, false};
  if ( lineSensorValues[0] < threshold[0])
    sensorsState.L = true;
  if ( lineSensorValues[1] < threshold[1]){
    sensorsState.LC = true;
    //Serial.println("L = true");
    }
  if ( lineSensorValues[2] < threshold[2]){
    sensorsState.C = true;
    //Serial.println("C = true");
    }
  if ( lineSensorValues[3] < threshold[3]){
    sensorsState.RC = true;
    //Serial.println("R = true");
    }
  if ( lineSensorValues[4] < threshold[4])
    sensorsState.R = true;
}




void driveDist (double dist) {
  double count = 0;
  //resets encoders to avoid encoder counts for e.g. turns
  //factor so that it can drive backwards
  int f = 1;
  if (dist < 0) f = -1;
    
    count = dist*78.8; //Don't ask, it just works
    
    
    //Following code prints on the lcd, the distance it is driving in cm
    lcd.gotoXY(0,0);
    lcd.print("Driving");
    lcd.gotoXY(0,1);
    lcd.print((String)dist + " cm");
   
    //actual function
    while(f*rememberCounts[0]< f*count) {
      location();
      testAccuracy();
      readSensors(sensorsState);
      //if loop so that it avoids driving unevenly 
      if(sensorsState.C == true && sensorsState.RC == true){ //f*rememberCounts[0]*1.001 > f*rememberCounts[1]
        // when right counter is higher, the left motor goes to 300 speed for a moment to even out
        motors.setSpeeds(f*130,f*100); 
        delay(1);

      } else if (sensorsState.LC == true && sensorsState.C == true){ //f*rememberCounts[0]*1.001 < f*rememberCounts[1]
        //Same as above but with other motor
        motors.setSpeeds(f*100,f*130);
        delay(1);
      }
      else if (sensorsState.LC == true){ //f*rememberCounts[0]*1.001 < f*rememberCounts[1]
        //Same as above but with other motor
        motors.setSpeeds(f*100,f*150);
        delay(1);}
        
        else if (sensorsState.RC == true){ //f*rememberCounts[0]*1.001 < f*rememberCounts[1]
        //Same as above but with other motor
        motors.setSpeeds(f*150,f*100);
        delay(1);}
      else {
        motors.setSpeeds(f*100,f*100);//standard speed
      }
    }
    //stops driving and clears LCD
    motors.setSpeeds(0, 0);
    lcd.clear();
}

void resetCounts() {//Function to reset an array that counts the encoders, this is because the encoders are also needed to calculate the location function
  rememberCounts[0] = 0;
  rememberCounts[1] = 0;

}

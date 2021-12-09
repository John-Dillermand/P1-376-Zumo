#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>
#include <Zumo32U4IMU.h>
#include <Arduino.h>
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LCD lcd;
Zumo32U4IMU imu;

double x = 0, y = 0, pose = 0, temp, deg = 0; //the x coordinate, the y coordinate, the position angle compared to the x-axis given in radians, "deg" For cenverting radians to degrees 
//Distance on the right and left will always come from an equal amount of time
double distRight, distLeft; //the dist traveled on the right wheel, dist traveled on left wheel, counts from 
int Position = 0; // The angle at which the robot is pointing in the global coordinate system

double poseVector[3] = {0,0,0};  // The "beginning" vector 
double newPose[3]; //A "new position" vector for new coordinates and angles from the distance traveled given that the starting position is [0,0,0]
int counts[2]; //an array for the numbers retrieved from the encoders
int rememberCounts[2] = {0,0}; //a vector to remember the counts 

int i = 0; //Iterations of the for-loop
int maxY = 50; //The max y-distance on the field
int wildOats[2][5] = {{0,7,12,23,39},{0,12,23,37,46}}; //An array describing the positions of the wild oats

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
    driveDist(wildOats[1][i+1]-wildOats[1][i]); //Then drives to the wild oats y-value
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
  if (poseVector[1] > (maxY/2)){
    turnLeft();
    returnYmax();
    turnStraight();
  }
  else{
   turnRight();
   returnTo0();
   turnStraight();
  }
}



void driveDist (double dist) {
  double count = 0;
  //resets encoders to avoid encoder counts for e.g. turns
  //factor so that it can drive backwards
  int f = 1;
  if (dist < 0) f = -1;
    
    count = dist*78.5; //Don't ask, it just works
    
    
    //Following code prints on the lcd, the distance it is driving in cm
    lcd.gotoXY(0,0);
    lcd.print("Driving");
    lcd.gotoXY(0,1);
    lcd.print((String)dist + " cm");
   
    //actual function
    while(f*rememberCounts[0]< f*count) {
      motors.setSpeeds(f*100,f*100);//standard speed
      location();
      //if loop so that it avoids driving unevenly 
      if((f*rememberCounts[0]*1.01) > (f*rememberCounts[1])){
        // when right counter is higher, the left motor goes to 300 speed for a moment to even out
        motors.setSpeeds(f*300,f*100); 

      } if (f*rememberCounts[0]*1.01 < f*rememberCounts[1]){
        //Same as above but with other motor
        motors.setSpeeds(f*100,f*300);
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

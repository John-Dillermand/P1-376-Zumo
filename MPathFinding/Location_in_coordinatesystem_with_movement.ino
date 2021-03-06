void calibrating(){ //calibraing gyro
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  lcd.clear();
}

void turning(int angle){ //Functiion that turns the robot to a specific angle in the global coordinate system
  imusetup();
  turn2(angle); //Makes it turn
  resetCounts(); //resests the encoders after turning
}

void turnStraight(){ //Short function that makes it look straight ahead (face parallell to the global x-axis)
  turning(0);
}

void turnBackward(){
  turning(-179); //if the x-coordinate of the next wild oat is less than the current x-position, it need to turn around to drive forward otherwise it reverses, which it can't do straight
}

void turnLeft(){//Short function that makes it turn to the left (face parallell to the global y-axis)
  turning(90);
}

void turnRight(){//Short function that makes it turn to the right (face parallell to the global y-axis, forward is just opposite of left)
  turning(-90);
}

void getEncoderAndDist() {
  counts[0] = encoders.getCountsAndResetRight(); //Getting the counts from each encoder
  counts[1] = encoders.getCountsAndResetLeft();
  rememberCounts[0] = rememberCounts[0]+counts[0]; //remembering the counts for the driveDist function
  rememberCounts[1] = rememberCounts[1]+counts[1];
  distRight = (1/78.8)*counts[0]; // Using these counts to calculate the distance each wheel has traveled 
  distLeft = (1/78.8)*counts[1]; // 1/78.5 is the ratio between a centimeter and 1 encoder count
}

void calCoordiPose(){
  double distTotal = 0; //Sort of a total distance traveled, but not really
  newPose[2] = Position*(M_PI/180); /*calculate the new position, given in radians
  usually you'd use the distance the wheels have traveled, but this was very inaccurate so we just use the gyro*/
  distTotal = (distRight+distLeft)/2; //calculating total distance so that a nex x and y can be calculated
  newPose[0] = distTotal*cos(poseVector[2]+(0/2)); //calculate the new x-coordinate after move
  newPose[1] = distTotal*sin(poseVector[2]+(0/2)); // calculate the new y-coordinate after move
  
  //newPose[0] = xNew; //Each new calculated value is given to the "new position" vector
  //newPose[1] = yNew;
  //newPose[2] = poseNew;
 
  x = poseVector[0] + newPose[0]; // adding new x-value to the old one
  y = poseVector[1] + newPose[1]; // adding new y-value to the old one
  temp = Position*(M_PI/180);
   //These new values are now the values of the next "beginning" vector

  Position = temp*(180/M_PI); 
  poseVector[0] =x; //Each new value is given to the position vector
  poseVector[1] =y;
  poseVector[2] =temp;
}

void location(){ //function compiling all the other functions
  getEncoderAndDist(); //Getting the distance each wheel has traveled from the encoder counts 
  calCoordiPose(); //using those distances to calculate the coordinates
  //convertRadians(); // converting the radians to degrees for the print
  Serial.println("New x-value: " + (String)poseVector[0] + " New y-value: " + (String)poseVector[1] + " New position: " + (String)Position + " degrees");
  delay(1); //A small delay for good measure
}

void driving(){ //function that does the whole driving thing
  continuing(); /*Assuming that it is on the either the x-axis or maxY and pointing the correct direction for the next point, 
  it then drives the distance needed to reach the new points x-value and then turns either right or left depending on wether the bot is on the x-axis or maxY*/
  delay(500);
  driveToSpot(); /*Assuming that it is pointing the correct way to reach the y-value of the point, it then drives the distance needed to reach that y-value 
  and when it has reached that value to it turns "forward" according to the global coodinate system (This is to be able to drive out into the field if we got that far)*/
  buzzer.playFrequency(1000,500,10); //Buzzer to show that it has reached a point
  delay(500);
  while(wildOats[0][i+1] == wildOats[0][i+2]){ //A while-loop that checks if the next point is on the same line and drives to it, so that it doesn't go down to the x-axis or maxY
    sameLine(); /*Assuming that the robot is already at a point, it will then check whether the next point's y-value is bigger or smaller than the current one
    then turn to the appropriate direction and drive to the next point's y-value*/
    delay(500);
  }
    returning(); /*Assuming that the robot is out in the field, it will calculate the shortest route and then drive to either the x-axis 
    or max y-value and then turn to the appropriate direction so that the "continuing" function can drive to the next point's x-value*/
    delay(500);
}


bool cali = true; //A bool that makes it calibrate the gyro once

void setup() {
  Serial.begin(9600);
  randomSeed(39); //Change the number to get different routes
  //using 10, gives a route that has a lot of points around the edge, giving a few problems
  //Using 12 pretty much starts off by turning a little too much and then not following the line
  lineSensors.initFiveSensors();
  readSensors(sensorsState);
  delay(5000);
  fillDestRand(); //Making random points 
  delay(500);
  routeCal('i'); //Calculating the shortest route
  delay(200);

}
void loop() {
  lcd.print("Press A");
  buttonA.waitForPress();
  lcd.clear();
  while (cali == true){ //While-statement that calibrates the gyro
    calibrating();
    cali = false;
  }
  for (i = 0; i < numDest; i++){ //For-loop to make it go through every wild oat
  driving();
  }
  delay(10000);
}
  

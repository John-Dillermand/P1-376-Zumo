
uint32_t turnAngle = 0;
int turnAngle2=turnAngle;
// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;


void turnSensorSetup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  lcd.clear();
  lcd.print(F("Gyro cal"));

  // Turn on the yellow LED in case the LCD is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  lcd.clear();
}


// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.


void turnSensorUpdate()
{
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dxt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}
//The amount of degrees the robot has turned
int turnAngleDegrees;

void imusetup()
{
  turnSensorUpdate();
  turnAngleDegrees=((((int32_t)turnAngle >> 16) * 360) >> 16);
  lcd.gotoXY(0, 0);
  lcd.print((((int32_t)turnAngle >> 16) * 360) >> 16);
  lcd.print(F("   "));
}


//Ik rør ved noget førhen, dette er alt sammen Carlos kode der bliver brugt til gyro
/* This should be called in setup() to enable and calibrate the
gyro.  It uses the LCD, yellow LED, and button A.  While the LCD
is displaying "Gyro cal", you should be careful to hold the robot
still.
The digital zero-rate level of the gyro can be as high as
25 degrees per second, and this calibration helps us correct for
that. */



void turn2(int angle) //The function takes in a angle and a speed
{
  bool finish = false;            //Used to check when the zumo has turned the angle degrees
  int f = 1;
  int speeds = 120;
  int angle2 = angle;
  if(Position>angle) f = -1; //Using the old position angle to determine which direction it should spin, to spin the least.
  if(Position-angle>180 || Position-angle<-180) f = -f;
  if(angle < 0){angle = angle + 2;} else if (angle > 0) {angle = angle - 2;}
  while(finish  !=  true)         //A while loop that runs untill the robot has turned the amount of degrees
  {
    if(turnAngleDegrees == (angle))
    {
      motors.setSpeeds(0, 0);
      finish  = true;
    }
    else
    {
      motors.setSpeeds(f*(-speeds), f*(speeds));        //Changes the rotation dependent on the given angle
      imusetup();
      if(f*rememberCounts[0]*1.01 > f*rememberCounts[1]){
        // when right counter is higher, the left motor goes to 200 speed for a moment to even out
        motors.setSpeeds(f*200,f*120); 

      } else if (f*rememberCounts[0]*1.01 < f*rememberCounts[1]){
        //Same as above but with other motor
        motors.setSpeeds(f*120,f*200);
      }
    }
  }
  Position = angle2; //Making the new position value equal the angle it has been told to turn to.
  if(Position == -179){
    Position = 180;
  }
  encoders.getCountsAndResetRight(); //just resetting the encoders so that it doesn't use them in the location function
  encoders.getCountsAndResetLeft();
  location(); //Just prints the new positions values after it has turned
}

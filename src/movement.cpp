#include "movement.h"
#include "devices.h"
#include "vex.h"
#include <iostream>
using namespace vex;

double turnSlewStep = 6;

double turnSlew(double val)
{
  static double prevVal = 0;
  if (prevVal + turnSlewStep < val)
  {
    prevVal = prevVal + turnSlewStep;
    return prevVal;
  }
  prevVal = val;
  return val;
}

void inert(double target, double kP = 0.499, double kI = 0, double kD = 0.002)
{
  bool isComplete = false;
  double startTime = Brain.timer(msec), prevTime = startTime, deltaTime, currentTime;
  double integral = 0, error = 0, derivative = 0, prevError = 0;
  double currentDeg = Inertial.heading(degrees);
  double output;
  int oscillation = 0;
  while (!isComplete)
  {
    //////////////////setting values
    currentTime = Brain.timer(sec);
    deltaTime = currentTime - prevTime;
    prevTime = currentTime;
    currentDeg = Inertial.heading(degrees);

    // calculations

    error = target - currentDeg;

    if (error > 180)
    {
      error = error - 360;
    }
    else if (error < -180)
    {
      error = 360 + error;
    }

    derivative = (error - prevError) / deltaTime;

    prevError = error;

    integral = +error * deltaTime;

    output = kP * error + kI * integral + kD * derivative;

    //  output = turnSlew(output);

    //////////////////////// motor output

    mBackLeft.spin(forward, output, percent);
    mFrontLeft.spin(forward, output, percent);
    mMidLeft.spin(forward, output, percent);
    mMidRight.spin(forward, -output, percent);
    mBackRight.spin(forward, -output, percent);
    mFrontRight.spin(forward, -output, percent);

    //      Printing values

    //      Exiting loop
    if (((fabs(prevError) < 1.5)))
    {
      oscillation++;
      if(oscillation > 1){
        isComplete = true;
        mBackLeft.stop(hold);
        mFrontLeft.stop(hold);
        mMidLeft.stop(hold);
        mBackRight.stop(hold);
        mFrontRight.stop(hold);
        mMidRight.stop(hold);
        break;
      }
    }
    vex ::wait(20, msec);
  }
}

void inertClamp(double t){
  inert(t, 0.499, 0.0, 0.002);
}

void driveDeg(int DDegL, int DDegR, int veloc)
{
  mBackLeft.setVelocity(veloc, percent);
  mBackRight.setVelocity(veloc, percent);
  mFrontLeft.setVelocity(veloc, percent);
  mFrontRight.setVelocity(veloc, percent);
  mMidLeft.setVelocity(veloc, percent);
  mMidRight.setVelocity(veloc, percent);

  mMidLeft.spinFor(forward, DDegL, degrees, false);
  mMidRight.spinFor(forward, DDegR, degrees, false);
  mBackLeft.spinFor(forward, DDegL, degrees, false);
  mBackRight.spinFor(forward, DDegR, degrees, false);
  mFrontLeft.spinFor(forward, DDegL, degrees, false);
  mFrontRight.spinFor(forward, DDegR, degrees, true);
}

void drivePID(double inches, double kP = 110, double kI = 0, double kD = .15, double goalThreshold = 30)
{
  // Function to control robot movement using PID
  int inGoal = 0;                                            // Tracks robot's time in goal threshold
  double currentDelta;                                       // Error between target and current position
  double P = 0, I = 0, D = 0, totalPID;                      // PID terms
  double pollingRate = 20;                                   // Polling rate in ms
  double target = inches * (360 / (2 * M_PI * 1.375));       // Target position in degrees (1.375 is wheelRadius)

  double previousDelta = target; // Initialize previous error as target
  double integralSum = 0;        // Cumulative error for integral term

  double startTime = Brain.Timer.time();
  double timeout = 3000; // max time before PID times out
  double goalsNeeded = (std::fabs(inches)/5) * pollingRate; // makes time spent in goal proportional to distance
  if(goalsNeeded == 0){ // sets bounds (max & min) for goals needed to reach goal
    goalsNeeded = 1;
  }
  else if(goalsNeeded > 5){
    goalsNeeded = 5;
  } 

  // Reset motor encoder value to 0
  mBackLeft.setPosition(0, degrees);
  mFrontLeft.setPosition(0, degrees);
  mMidLeft.setPosition(0, degrees);
  mFrontRight.setPosition(0, degrees);
  mBackRight.setPosition(0, degrees);
  mMidRight.setPosition(0, degrees);



  while (inGoal < goalsNeeded)  // CHECK IF IT SHOULD BE A < or <=
  {
    // Main PID loop; runs until target is reached
    // Read motor position (you can average left and right motor values for straight driving)
    double currentPosition = (mBackLeft.position(degrees) + mBackRight.position(degrees) + mFrontLeft.position(degrees) + mMidLeft.position(degrees) + mMidRight.position(degrees) + mFrontRight.position(degrees)) / 6;

    // Calculate the current error
    currentDelta = target - currentPosition;

    // Proportional: Larger error results in larger response
    P = (kP / 1000) * currentDelta;

    // Integral: Sum of all errors helps correct for small errors over time
    integralSum += currentDelta;
    I = kI * integralSum;

    // Derivative: React to the rate of error change
    D = kD * (currentDelta - previousDelta) / pollingRate;

    // Calculate total PID response
    totalPID = P + I + D;

    // Use totalPID to move motors proportionally
    mBackLeft.spin(forward, totalPID, percent);
    mFrontLeft.spin(forward, totalPID, percent);
    mMidLeft.spin(forward, totalPID, percent);
    mMidRight.spin(forward, totalPID, percent);
    mBackRight.spin(forward, totalPID, percent);
    mFrontRight.spin(forward, totalPID, percent);

    // Check if the error is small enough to stop
    if (fabs(currentDelta) < goalThreshold)
    {
      inGoal
    }
    else{
      inGoal = 0;
    }
    // Check if should timeout
    if((Brain.Timer.time() - startTime) >= timeout){
      break;
    }

    // Update the previous error for the next loop
    previousDelta = currentDelta;

    // Wait for the polling rate before next iteration
    wait(pollingRate, msec);
  }
  // Stop the motors once goal is met
  mBackLeft.stop();
  mBackRight.stop();
  mFrontLeft.stop();
  mFrontRight.stop();
  mMidLeft.stop();
  mMidRight.stop();
}

void tunerDrivePID(double inches, double kP = 110, double kI = 0, double kD = .15, int ID = -1)
{
  // Function to control robot movement using PID
  int goalMet = 0;                                     // Flag to track if the goal is met
  double currentDelta;                                 // Error between target and current position
  double P = 0, I = 0, D = 0, totalPID;                // PID terms
  double pollingRate = 20;                             // Polling rate in ms
  double target = inches * (360 / (2 * M_PI * 1.375)); // Target position in degrees
  double startTime = Brain.Timer.time();               // TUNER EXCLUSIVE: starting time for timeout
  double timeElapsed = 0;                              // TUNER EXCLUSIVE: time elapsed since starting PID
  double currentPitch = Inertial.pitch();
  double maxPitch = 0;

  double previousDelta = target; // Initialize previous error as target
  double integralSum = 0;        // Cumulative error for integral term

  std::cout << "target = " << target << std::endl; // TUNER EXCLUSIVE: prints goal to console for graphing

  // Reset motor encoder value to 0
  mBackLeft.setPosition(0, degrees);
  mFrontLeft.setPosition(0, degrees);
  mMidLeft.setPosition(0, degrees);
  mFrontRight.setPosition(0, degrees);
  mBackRight.setPosition(0, degrees);
  mMidRight.setPosition(0, degrees);

  while (goalMet <= 1 && timeElapsed <= 5000)
  { // Main PID loop; runs until target is reached
    // Read motor position (you can average left and right motor values for straight driving)
    double currentPosition = (mBackLeft.position(degrees) + mBackRight.position(degrees) + mFrontLeft.position(degrees) + mMidLeft.position(degrees) + mMidRight.position(degrees) + mFrontRight.position(degrees)) / 6;

    // Calculate the current error
    currentDelta = target - currentPosition;

    // Proportional: Larger error results in larger response
    P = (kP / 1000) * currentDelta;

    // Integral: Sum of all errors helps correct for small errors over time
    integralSum += currentDelta;
    I = kI * integralSum;

    // Derivative: React to the rate of error change
    D = kD * (currentDelta - previousDelta) / pollingRate;

    // Calculate total PID response
    totalPID = P + I + D;

    // Use totalPID to move motors proportionally
    mBackLeft.spin(forward, totalPID, percent);
    mFrontLeft.spin(forward, totalPID, percent);
    mMidLeft.spin(forward, totalPID, percent);
    mMidRight.spin(forward, totalPID, percent);
    mBackRight.spin(forward, totalPID, percent);
    mFrontRight.spin(forward, totalPID, percent);

    // Check if the error is small enough to stop
    if (fabs(currentDelta) < 30)
    {
      goalMet++;
    }
    // Update the previous error for the next loop
    previousDelta = currentDelta;

    // TUNER EXCLUSIVE: time elapsed for timeout
    timeElapsed = Brain.Timer.time() - startTime;

    // TUNER EXCLUSIVE: Print coordinates to console for graphing
    if (ID == -1)
    {
      std::cout << "(" << timeElapsed
                << "," << currentPosition << "," << currentPitch << ")" << std::endl;
    }
    else
    { // GRAPH EXCLUSIVE: finds maximum pitch of robot
      if (currentPitch > maxPitch)
      {
        maxPitch = currentPitch;
      }
    }

    // Wait for the polling rate before next iteration
    wait(pollingRate, msec);
  }
  // Stop the motors once goal is met
  mBackLeft.stop();
  mBackRight.stop();
  mFrontLeft.stop();
  mFrontRight.stop();
  mMidLeft.stop();
  mMidRight.stop();
  // GRAPH EXCLUSIVE: print coordinates for advanced graphing
  if (ID != -1)
  {
    double maxPitch = 1; // TEMP
    double motorHeat = (mBackLeft.position(degrees) + mBackRight.position(degrees) + mFrontLeft.position(degrees) + mMidLeft.position(degrees) + mMidRight.position(degrees) + mFrontRight.position(degrees)) / 6;
    std::cout << "(" << ID << "," << kP << "," << kD << ","
              << timeElapsed << "," << maxPitch << ","
              << motorHeat << std::endl;
  }
}

void odometry(){


  // Initialize starting values
  double leftPos, rightPos, perpPos;
  double prevLeftPosition = 0 , prevRightPosition = 0;
  double deltaLeft, deltaRight;
  double totalDeltaLeft = 0, totalDeltaRight = 0;
  double theta = Intertial.heading() * math.pi / 180; // only working with radians in odom
  double globalX = 0, globalY = 0;

  bool odomRunning = true;
  while(odomRunning){
    leftPos = mMidLeft.position();
    rightPos = mMidRight.position();


  }





}

void oldDrivePID(double degrs, double veloc)
{
  double average = 0;
  // reset postion to zero
  mBackLeft.setPosition(0, degrees);
  mFrontLeft.setPosition(0, degrees);
  mMidLeft.setPosition(0, degrees);
  mFrontRight.setPosition(0, degrees);
  mBackRight.setPosition(0, degrees);
  mMidRight.setPosition(0, degrees);
  if (degrs > 0)
  {
    while (average < degrs)
    {
      // spins while less than(average is the position of all the motors averages)
      mBackLeft.spin(forward, veloc, pct);
      mMidLeft.spin(forward, veloc, pct);
      mFrontLeft.spin(forward, veloc, pct);
      mBackRight.spin(forward, veloc, pct);
      mMidRight.spin(forward, veloc, pct);
      mFrontRight.spin(forward, veloc, pct);
      average = (mBackLeft.position(degrees) + mBackRight.position(degrees) + mFrontLeft.position(degrees) + mMidLeft.position(degrees) + mMidRight.position(degrees) + mFrontRight.position(degrees)) / 6;
      // slows down in last 100
      if (veloc > 15 && (average) > (degrs - 80))
      {
        veloc = veloc * ((degrs - average) / degrs);
      }
      vex ::wait(10, msec);
    }
  }
  else if (degrs < 0)
  {
    while (average > degrs)
    {
      // same thing for other direction
      mBackLeft.spin(reverse, veloc, pct);
      mMidLeft.spin(reverse, veloc, pct);
      mFrontLeft.spin(reverse, veloc, pct);
      mBackRight.spin(reverse, veloc, pct);
      mMidRight.spin(reverse, veloc, pct);
      mFrontRight.spin(reverse, veloc, pct);
      average = (mBackLeft.position(degrees) + mBackRight.position(degrees) + mFrontLeft.position(degrees) + mMidLeft.position(degrees) + mMidRight.position(degrees) + mFrontRight.position(degrees)) / 6;
      if (veloc > 15 && (average) < (degrs + 100))
      {
        veloc = veloc * ((degrs - average) / degrs);
      }
      vex ::wait(10, msec);
    }
  }
  /*
    mBackLeft.stop(hold);
    mBackRight.stop(hold);
    mFrontLeft.stop(hold);
    mFrontRight.stop(hold);
    mMidLeft.stop(hold);
    mMidRight.stop(hold);*/
  vex ::wait(200, msec);
}
void drivePIDClamp(double degrs, double veloc)
{
  double average = 0;
  bool clampPOS = true;
  // reset postion to zero
  mBackLeft.setPosition(0, degrees);
  mFrontLeft.setPosition(0, degrees);
  mMidLeft.setPosition(0, degrees);
  mFrontRight.setPosition(0, degrees);
  mBackRight.setPosition(0, degrees);
  mMidRight.setPosition(0, degrees);
  if (degrs > 0)
  {
    while (average < degrs)
    {
      // spins while less than(average is the position of all the motors averages)
      mBackLeft.spin(forward, veloc, pct);
      mMidLeft.spin(forward, veloc, pct);
      mFrontLeft.spin(forward, veloc, pct);
      mBackRight.spin(forward, veloc, pct);
      mMidRight.spin(forward, veloc, pct);
      mFrontRight.spin(forward, veloc, pct);
      average = (mBackLeft.position(degrees) + mBackRight.position(degrees) + mFrontLeft.position(degrees) + mMidLeft.position(degrees) + mMidRight.position(degrees) + mFrontRight.position(degrees)) / 6;
      // slows down in last 100
      if (veloc > 15 && (average) > (degrs - 250))
      {
        veloc = veloc * ((degrs - average) / degrs);
        if (clampPOS == true)
        {
          sClamp.set(true);
          clampPOS = true;
        }
      }
      vex ::wait(10, msec);
    }
  }
  else if (degrs < 0)
  {
    while (average > degrs)
    {
      // same thing for other direction
      mBackLeft.spin(reverse, veloc, pct);
      mMidLeft.spin(reverse, veloc, pct);
      mFrontLeft.spin(reverse, veloc, pct);
      mBackRight.spin(reverse, veloc, pct);
      mMidRight.spin(reverse, veloc, pct);
      mFrontRight.spin(reverse, veloc, pct);
      average = (mBackLeft.position(degrees) + mBackRight.position(degrees) + mFrontLeft.position(degrees) + mMidLeft.position(degrees) + mMidRight.position(degrees) + mFrontRight.position(degrees)) / 6;
      if (veloc > 15 && (average) < (degrs + 150))
      {
        veloc = veloc * ((degrs - average) / degrs);
        if (clampPOS == true)
        {
          sClamp.set(false);
          clampPOS = false;
        }
      }
      vex ::wait(10, msec);
    }
  }
  /*
    mBackLeft.stop(hold);
    mBackRight.stop(hold);
    mFrontLeft.stop(hold);
    mFrontRight.stop(hold);
    mMidLeft.stop(hold);
    mMidRight.stop(hold);*/
}
void driveInches(double fwdVal, int veloc)
{
  const double diameter = 2.75;
  const double pi = 3.1415;
  const double input = 36;
  const double output = 48;

  double num = fwdVal;
  double denom = (diameter * pi) * (input / output);
  double fwdInches = num / denom * 360;

  oldDrivePID(fwdInches, veloc);
}
void driveInchesClamp(double fwdVal, int veloc)
{
  const double diameter = 2.75;
  const double pi = 3.1415;
  const double input = 36;
  const double output = 48;

  double num = fwdVal;
  double denom = (diameter * pi) * (input / output);
  double fwdInches = num / denom * 360;

  drivePIDClamp(fwdInches, veloc);
}

void tunePID()
{
  using std::cout;
  using std::endl;
  double tempPID[3] = {110, 0, 0.15};           // Starting values for PID
  const double resetVal = 0.1, resetMag = 0.1;  // Defines what values to reset to
  double valMag = resetMag;                     // Sets initial magnitude
  int currentConst = 0;                         // Defines current constant (P,I, or D) to change
  cout << "Starting P: " << tempPID[0] << endl; // logs starting values to console
  cout << "Starting I: " << tempPID[1] << endl;
  cout << "Starting D: " << tempPID[2] << endl;
  cout << "Now modifying P" << endl; // tells user default value modified
  updateController(tempPID[0], tempPID[1], tempPID[2], 0, valMag);
  while (true)
  {

    if (Controller1.ButtonUp.pressing() && Controller1.ButtonDown.pressing())
    {
      valMag = resetMag; // Resets magnitude of dynamic change
      cout << "magnitude RESET to " << resetMag << endl;
      updateController(resetMag);
    }
    else if (Controller1.ButtonUp.pressing())
    {
      valMag *= 10; // Increments magnitude by a factor of 10
      cout << "magnitude set to " << valMag << endl;
      updateController(valMag);
    }
    else if (Controller1.ButtonDown.pressing())
    {
      valMag /= 10; // Decrements magnitude by a factor of 10
      cout << "magnitude set to " << valMag << endl;
      updateController(valMag);
    }

    if (Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing())
    {
      tempPID[currentConst] = resetVal; // Resets temp. PID value
      cout << "value RESET to " << resetVal << endl;
      updateController(resetVal, currentConst, valMag);
    }
    else if (Controller1.ButtonLeft.pressing())
    {
      tempPID[currentConst] -= valMag; // Decreases temp. PID value by magnitude
      cout << "value set to " << tempPID[currentConst] << endl;
      updateController(tempPID[currentConst], currentConst, valMag);
    }
    else if (Controller1.ButtonRight.pressing())
    {
      tempPID[currentConst] += valMag; // Increases temp. PID value by magnitude
      cout << "value set to " << tempPID[currentConst] << endl;
      updateController(tempPID[currentConst], currentConst, valMag);
    }

    if (Controller1.ButtonY.pressing())
    { // Controls value being changed
      currentConst += 1;
      currentConst %= 3; // variable has 3 states, modulus keeps within 3 states
      if (currentConst == 0)
      {
        cout << "Now modifying P" << endl;
      }
      else if (currentConst == 1)
      {
        cout << "Now modifying I" << endl;
      }
      else
      {
        cout << "Now modifying D" << endl;
      }
      updateController(tempPID[0], tempPID[1], tempPID[2], currentConst, valMag);
    }
    if (Controller1.ButtonA.pressing())
    { // Starts test with the given variables
      cout << "Starting test with following P,I,D: " << tempPID[0] << "," << tempPID[1] << "," << tempPID[2] << endl;
      tunerDrivePID(24 * 2, tempPID[0], tempPID[1], tempPID[2]); // Tests PID by moving 2 tiles WITH LOGGING
      wait(2, sec);
      inert(Inertial.rotation() + 180); // Turns robot so it is repeatable
      wait(1, sec);
      cout << "Test complete" << endl;
    }
    wait(100, msec); // pause between input detection to not double them accidentally
  }
}

void GraphPID(double rangeP, double rangeD, double guessP, double guessD, int sqrtTests)
{
  double currentP = guessP - rangeP;  // Values representing current search boundaries
  double currentD = guessD - rangeD;
  double minP = currentP;             // Search starts at the minimum, goes to maximum
  double deltaP = rangeP / sqrtTests;
  double deltaD = rangeD / sqrtTests;
  int maxID = sqrtTests ^ 2 - 1;
  int ID = 0;                    // IDs represent individual tests to make them replicable
  std::cout << "What is current ID (0 if starting): ";
  std::cin >> ID;            // retrieves user input so it can be run in mutliple sessions
  std::cout << std::endl;
  while (ID <= maxID)
  {

    tunerDrivePID(100, currentP, 0, currentD, ID); // Custom PID with more logging

    if (ID % 10 == 9)     // This is the loop that creates the gridlike pattern
    {
      currentD += deltaD;
      currentP = minP;
    }
    else
    {
      currentP += deltaP;
    }
    ID++;         // assigns new ID so next test can start
  }
}

void setArm(int armPos)
{
  double targetDeg;
  if(armPos==1){
    targetDeg = 0;
  }
  else if(armPos==2){
    targetDeg = 25;
  }
  else if(armPos==3){
    targetDeg = 130;
  }

  // Function to control robot movement using PID
        double P, I, D, totalPID;
        double kP = 1500;
        double kI = 0;
        double kD = 0.01;                     // Flag to track if the goal is met
        double currentDelta;                  // Error between target and current position
        P = 0, I = 0, D = 0; // PID terms            // Polling rate in ms // Target position in degrees
        int goalMet = 0;              // Flag to track if the goal is met      // Error between target and current position
         // PID terms      // Polling rate in ms // Target position in degrees

        double previousDelta; // Initialize previous error as target
        double integralSum;     // Cumulative error for integral term

        double currentPosition = Rotation.angle(deg); 
  // Reset motor encoder value to 0
  mLift.setPosition(0, degrees);

  while (goalMet <= 1)
        {
          // Main PID loop; runs until target is reached
          // Read motor position (you can average left and right motor values for straight driving)
          double currentPosition = Rotation.angle(deg);
          double error = targetDeg - currentPosition;

          if(error>180){
            error-=360;
          }
          else if(error<-180){
            error+=360;
          }

          // Calculate the current error
          currentDelta = error;

          // Proportional: Larger error results in larger response
          P = (kP / 1000) * currentDelta;

          // Integral: Sum of all errors helps correct for small errors over time
          integralSum += currentDelta;
          I = kI * integralSum;

          // Derivative: React to the rate of error change
          D = kD * (currentDelta - previousDelta) / 20;//

          // Calculate total PID response
          totalPID = P + I + D;

          // Use totalPID to move motors proportionally
          mLift.spin(forward, totalPID, percent); //THIS MIGHT NEED TO BE REVERSED
          mLift2.spin(forward, totalPID, percent);
          // Check if the error is small enough to stop
          if (fabs(currentDelta) < 2)
          {
            goalMet++;
          }
          // Update the previous error for the next loop
          previousDelta = currentDelta;
          // Wait for the polling rate before next iteration
          // changed to nothing
          wait(20,msec);
        }
  // Stop the motors once goal is met
  mLift.stop(hold);
  mLift.stop(hold);
}
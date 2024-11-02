/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// mBackRight           motor         20
// mBackLeft            motor         9
// mFrontRight          motor         1
// mFrontLeft           motor         2
// Controller1          controller
// mIntake              motor         3
// mPuncher             motor         6
// Inertial             inertial      17
// sWing                digital_out   G
// shotBlock            digital_out   F
// mMidLeft             motor         14
// mMidRight            motor         5
// CataStop             limit         A
// FrontWings           digital_out   B
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>
using namespace vex;
competition Competition;
brain Brain;
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
// controller Controller2 = controller();

// Motor and sensor device declarations
motor mBackRight = motor(PORT7, ratio6_1, false);
motor mBackLeft = motor(PORT10, ratio6_1, true);
motor mFrontRight = motor(PORT5, ratio6_1, false);
motor mFrontLeft = motor(PORT8, ratio6_1, true);
motor mMidLeft = motor(PORT9, ratio6_1, true);
motor mMidRight = motor(PORT6, ratio6_1, false);
motor mIntake = motor(PORT1, ratio6_1, false);
motor mLift = motor(PORT3, ratio18_1, true);
motor mLift2 = motor(PORT4, ratio18_1, false);
distance ringCheck = distance(PORT17);
inertial Inertial = inertial(PORT11);
rotation Rotation = rotation(PORT2);

// Digital-out device declarations
digital_out sClamp = digital_out(Brain.ThreeWirePort.B);
digital_out sDoor = digital_out(Brain.ThreeWirePort.F);
digital_out sintake = digital_out(Brain.ThreeWirePort.D);


// Limit switch declarations
limit limitS = limit(Brain.ThreeWirePort.E);



int auton = 0;
void autonSelection()
{
  if (auton <= 7)
  {
    auton++;
  }
  else
  {
    auton = 0;
  }
  switch (auton)
  {
  case 0:
    Brain.Screen.clearLine();
    Brain.Screen.print("Right side");
    break;
  case 1:
    Brain.Screen.clearLine();
    Brain.Screen.print("Red Left Auto");
    break;
  case 2:
    Brain.Screen.clearLine();
    Brain.Screen.print("Blue Right Auto");
    break;
  case 3:
    Brain.Screen.clearLine();
    Brain.Screen.print("Skills");
    break;
  case 4:
    Brain.Screen.clearLine();
    Brain.Screen.print("AWP");
    break;
  case 5:
    Brain.Screen.clearLine();
    Brain.Screen.print("Half AWP");
    break;
  case 6:
    Brain.Screen.clearLine();
    Brain.Screen.print("adaptive");
    break;
  case 7:
    Brain.Screen.clearLine();
    Brain.Screen.print("blue Mid Auto");
    break;
  }
}

void pre_auton(void)
{
  sClamp.set(true);
  Brain.Screen.pressed(autonSelection);

  Inertial.calibrate();
  vex ::wait(4, sec);
}

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
/*void odomleft(int rad, int degree){


double lWheelRad = (rad+5.25);

double rWheelRad = (rad-5.25);

double lWheelCirc = ((2*lWheelRad*3.14*degree/360) /360*3.14*3);

double rWheelCirc = ((2*rWheelRad*3.14*degree/360) /360*3.14*3);

double rWheelSpeed = rWheelCirc/lWheelCirc;

  mBackLeft.setVelocity(100,percent);
  mBackRight.setVelocity(rWheelSpeed,percent);
  mMidRight.setVelocity(rWheelSpeed,percent);
  mFrontLeft.setVelocity(100,percent);
  mFrontRight.setVelocity(rWheelSpeed,percent);
  mMidLeft.setVelocity(100,percent);

  mMidLeft.spinFor(forward,lWheelCirc,degrees,false);
  mMidRight.spinFor(forward,rWheelCirc,degrees, false);
  mBackLeft.spinFor(forward,lWheelCirc,degrees,false);
  mBackRight.spinFor(forward,rWheelCirc,degrees,false);
  mFrontLeft.spinFor(forward,lWheelCirc,degrees,false);
  mFrontRight.spinFor(forward,rWheelCirc,degrees,true);
}*/
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
/*void inert(double turnDeg){
    mBackLeft.setVelocity(speed,percent);
  mBackRight.setVelocity(speed,percent);
  mFrontLeft.setVelocity(speed,percent);
  mFrontRight.setVelocity(speed,percent);
  mMidLeft.setVelocity(speed,percent);
  mMidRight.setVelocity(speed,percent);
  if(Inertial.rotation()>turnDeg){
    double dif = Inertial.rotation()-turnDeg;
    while(Inertial.rotation()>turnDeg){
      mBackLeft.spin(reverse,speed,pct);
      mMidLeft.spin(reverse,speed,pct);
      mFrontLeft.spin(reverse,speed,pct);
      mBackRight.spin(forward,speed,pct);
      mMidRight.spin(forward,speed,pct);
      mFrontRight.spin(forward,speed,pct);
      if(speed>10){
          speed = speed*(((Inertial.rotation()-turnDeg))/dif);
      }
      Brain.Screen.clearLine();
      Brain.Screen.print(Inertial.rotation());
      vex :: wait(10,msec);
    }
    mBackLeft.stop(hold);
    mBackRight.stop(hold);
    mFrontLeft.stop(hold);
    mFrontRight.stop(hold);
    mMidLeft.stop(hold);
    mMidRight.stop(hold);
    vex :: wait(10,msec);

    while(Inertial.rotation()<turnDeg){
      mBackLeft.spin(forward,10,pct);
      mMidLeft.spin(forward,10,pct);
      mFrontLeft.spin(forward,10,pct);
      mBackRight.spin(reverse,10,pct);
      mMidRight.spin(reverse,10,pct);
      mFrontRight.spin(reverse,10,pct);
    }
    mBackLeft.stop(hold);
    mBackRight.stop(hold);
    mFrontLeft.stop(hold);
    mFrontRight.stop(hold);
    mMidLeft.stop(hold);
    mMidRight.stop(hold);
  }
  else if(Inertial.rotation()<turnDeg){
    double dif = turnDeg -Inertial.rotation();
    while(Inertial.rotation()<turnDeg){
      mBackLeft.spin(forward,speed,pct);
      mMidLeft.spin(forward,speed,pct);
      mFrontLeft.spin(forward,speed,pct);
      mBackRight.spin(reverse,speed,pct);
      mMidRight.spin(reverse,speed,pct);
      mFrontRight.spin(reverse,speed,pct);
      if(speed>10){
          speed = speed*(3/4)*((turnDeg-Inertial.rotation())/dif);
      }
      vex :: wait(10,msec);
    }
    while(Inertial.rotation()>turnDeg){
      mBackLeft.spin(reverse,10,pct);
      mMidLeft.spin(reverse,10,pct);
      mFrontLeft.spin(reverse,10,pct);
      mBackRight.spin(forward,10,pct);
      mMidRight.spin(forward,10,pct);
      mFrontRight.spin(forward,10,pct);
    }
    mBackLeft.stop(hold);
    mBackRight.stop(hold);
    mFrontLeft.stop(hold);
    mFrontRight.stop(hold);
    mMidLeft.stop(hold);
    mMidRight.stop(hold);
    Brain.Screen.print("monkey");
  }
        Brain.Screen.clearLine();
      Brain.Screen.print(Inertial.rotation());
}*/
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

/*
The overloaded function updateController is optimized
to only change the values necessary for increased responsiveness.
1. Updates whole screen (most time-consuming)
2. Updates magnitude (least time-consuming)
3. Changes one line (somewhat time-consuming)
*/
// for changing whole screen
void updateController(double kP, double kI, double kD, int sel, double mag)
{
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(sel + 1, 1);
  Controller1.Screen.print("*"); // creates marker for current selected value
  Controller1.Screen.setCursor(1, 2);
  Controller1.Screen.print(kP);
  Controller1.Screen.setCursor(2, 2);
  Controller1.Screen.print(kI);
  Controller1.Screen.setCursor(3, 2);
  Controller1.Screen.print(kD); // prints P I D on new lines
  Controller1.Screen.setCursor(2, 14);
  Controller1.Screen.print(mag);
}
// for changing just magnitude
void updateController(double mag)
{
  Controller1.Screen.setCursor(2, 14);
  Controller1.Screen.print(mag);
}
// for changing one line
void updateController(double val, double sel, double mag)
{
  sel += 1;
  Controller1.Screen.clearLine(sel);
  Controller1.Screen.setCursor(sel, 1);
  Controller1.Screen.print("*");
  Controller1.Screen.setCursor(sel, 2);
  Controller1.Screen.print(val);
  if (sel == 2)
  {
    updateController(mag);
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
  double currentP = guessP - rangeP;
  double currentD = guessD - rangeD;
  double minP = currentP;
  double deltaP = rangeP / sqrtTests;
  double deltaD = rangeD / sqrtTests;
  int maxID = sqrtTests ^ 2 - 1;
  int ID = 0;
  std::cout << "What is current ID (0 if starting): ";
  std::cin >> ID;
  std::cout << std::endl;
  while (ID <= maxID)
  {

    tunerDrivePID(100, currentP, 0, currentD, ID);

    if (ID % 10 == 9)
    {
      currentD += deltaD;
      currentP = minP;
    }
    else
    {
      currentP += deltaP;
    }
    ID++;
  }
}

void setArm(double targetDeg)
{
  // Function to control robot movement using PID
  double kP = 99.5;
  double kI = 0;
  double kD = .15;
  int goalMet = 0;                      // Flag to track if the goal is met
  double currentDelta;                  // Error between target and current position
  double P = 0, I = 0, D = 0, totalPID; // PID terms
  double pollingRate = 20;              // Polling rate in ms // Target position in degrees

  double previousDelta = targetDeg; // Initialize previous error as target
  double integralSum = 0;           // Cumulative error for integral term

  // Reset motor encoder value to 0
  mLift.setPosition(0, degrees);

  if (goalMet <= 1)
  {
    // Main PID loop; runs until target is reached
    // Read motor position (you can average left and right motor values for straight driving)
    double currentPosition = Rotation.angle(deg);

    // Calculate the current error
    currentDelta = targetDeg - currentPosition;

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
    mLift.spin(forward, totalPID, percent);

    // Check if the error is small enough to stop
    if (fabs(currentDelta) < 30)
    {
      goalMet++;
    }
    // Update the previous error for the next loop
    previousDelta = currentDelta;

    // Wait for the polling rate before next iteration
    // changed to nothing
  }
  // Stop the motors once goal is met
  mLift.stop(hold);
}

void skillsAuto()
{

  // Grab First Ring
  // std::cout<<"test"<<std::endl;

  drivePIDClamp(-200, 40);

 // lift the lift so it doesnt hit walls
  // drive and clamp

  mIntake.spin(fwd, 100, pct);
wait(700,msec);
inert(100);
drivePID(-70);
  sClamp.set(true);
  wait(500,msec);
  drivePID(20);
 /*
mIntake.spin(fwd,100,pct);
wait(700,msec);
drivePID(3);
inert(-95);
drivePIDClamp(-1000,60);
inert(90);
drivePID(30);
inert(-45);

*/

  /*
  mLift.spin(reverse,100,pct);
  driveInchesClamp(-6,60);
  wait(200,msec);
  mIntake.spin(fwd,100,pct);
  mLift.stop(brakeType::hold);
  inert(-90);
  drivePID(41);
  drivePID(-11);
  inert(0);
  drivePID(6);
  drivePID(-6);
  inert(135);
  drivePID(-20);
  sClamp.set(true);
  wait(200,msec);
  drivePID(24);
  inert(90);
  drivePID(60);
  inert(-90);
  driveInchesClamp(-25,65);
  */
}
void blueRightAuto(int i) // i is if its inversed or not (1 = regular, -1 = inversed)
{
  // this auto is intended to score 5 rings in one goal and score one on a side stake
  // Grab Goal
  drivePIDClamp(-1320, 50);
  // Turn towards mid ring stack
  inert(i*-140);
  mIntake.spin(forward, 100, pct); // Turn on intake
  // Pick up bottom ring
  drivePID(27);
  wait(200, msec);
  // Back up to avoid intaking second ring
  drivePID(-7);
  // Turn towards Alliance side one stack
  inert(i*-42);
  // waiting for ring to intake
  wait(300, msec);
  // Pick up red side one stack
  drivePID(17);
  wait(500,msec);
  inert(i*196); // turn towards other mid ring
  
drivePID(18.5);
inert(i*180);
drivePID(-56);
wait(300,msec);
inert(i*-45);
sintake.set(true);
drivePID(40);
sintake.set(false);
drivePID(-15);

  /*
  
  // Turn to second mid ring stack
  inert(0);
  // Wait for 0 turning velocity
  wait(50, msec);
  // Pick up second mid ring
  drivePID(18);
  // Turn to Alliance side two stack Perpendicular to side stake
  inert(-165);
  // Pick up Alliace side two stack Perpendicular to side stake
  drivePID(25);
  // Back up to avoid picking up bottom blue ring
  drivePID(-5);
  //
  inert(46);
  sintake.set(true);
  mIntake.spin(fwd, 60, pct);
  drivePID(68);
  sintake.set(false);
  oldDrivePID(-10, 50);
  mIntake.stop();
  inert(27);
  sDoor.set(true);
  mLift.spinFor(reverse, 300, degrees);
  drivePID(7);
  drivePID(13.5);
  mLift.spinFor(forward, 230, degrees);
  mLift.stop(coast);
  mIntake.spin(fwd, 100, pct);
  drivePID(-20);
  */
}
void blueMidAuto(){
  drivePID(-15.75);
  inert(-92);
  drivePID(-4);
  mIntake.spin(forward, 100, pct);
  wait(1000,msec);
  mIntake.stop(); 
    drivePID(3);
    
    inert(-145);
    mIntake.spin(fwd,100,pct);
drivePID(85);
inert(187);
mIntake.stop();
drivePIDClamp(-1400,50);

mIntake.spin(fwd,100,pct);
wait(500,msec);
inert(100);
drivePID(40);
inert(170);
sintake.set(true);
drivePID(55);
sintake.set(false);
drivePID(-56);
inert(-30);
drivePID(30);
/*sClamp.set(true);
drivePID(10);

inert(65);
drivePIDClamp(-1800,100);*/

}
void redleftAuto()
{
  // this auto is intended to score 5 rings
  // in one goal and score one on a side stake
  // Grab Goal
  drivePIDClamp(-1100, 80);
  // Turn on intake
  mIntake.spin(forward, 100, pct);
  // Turn towards mid ring stack
  inert(140);
  // Pick up bottom ring
  drivePID(25);
  wait(200, msec);
  // Back up to avoid intaking second ring
  drivePID(-5);
  // Turn towards Alliance side one stack
  inert(50);
  // waiting for ring to intake
  wait(300, msec);
  // Pick up red side one stack
  drivePID(10);
  // Turn to second mid ring stack
  inert(0);
  // Wait for 0 turning velocity
  wait(50, msec);
  // Pick up second mid ring
  drivePID(18);
  // Turn to Alliance side two stack
  // Perpendicular to side stake
  inert(165);
  // Pick up Alliace side two stack
  // Perpendicular to side stake
  drivePID(25);
  // Back up to avoid picking up bottom blue ring
  drivePID(-5);
  // turn towards stack of two and lift intake
  inert(-46);
  sintake.set(true);
  mIntake.spin(fwd, 60, pct);
  drivePID(68);
  sintake.set(false);
  // put ring on alliance stake after
  // turning and lift life
  oldDrivePID(-10, 50);
  mIntake.stop();
  inert(-27);
  sDoor.set(true);
  mLift.spinFor(reverse, 300, degrees);
  drivePID(7);
  // put rin on alliance stake and set
  // braketype coast
  drivePID(13.5);
  mLift.spinFor(forward, 230, degrees);
  mLift.stop(coast);
  mIntake.spin(fwd, 100, pct);
  drivePID(-20);
}
void rightAuto()
{

  sDoor.set(true);
  // Grab First Ring
  mIntake.spin(forward, 100, pct);
  drivePID(50); // pick up ring
  wait(300, msec);
  mIntake.stop();
  wait(100, msec);
  inert(160);
  driveInchesClamp(-24, 80); // grab goal
}
void AWP()
{
    // this auto is intended to score 5 rings in one goal and score one on a side stake
  // Grab Goal
  drivePIDClamp(-1250, 80);
  // Turn towards mid ring stack
  inert(-140);
  mIntake.spin(forward, 100, pct); // Turn on intake
  // Pick up bottom ring
  drivePID(24);
  wait(200, msec);
  // Back up to avoid intaking second ring
  drivePID(-5);
  // Turn towards Alliance side one stack
  inert(-42);
  // waiting for ring to intake
  wait(300, msec);
  // Pick up red side one stack
  drivePID(17);
  inert(65);//turn towards ring by allinace stake
  sintake.set(true);
  drivePID(57);
  sintake.set(false);//grabs ring by alliance stake
  wait(200,msec);
  drivePID(1);
  sClamp.set(true);//drops goal
  drivePID(-15);//backs up so blue isnt picked up
  mIntake.stop();
  inert(0);
  drivePID(30);
  inert(90);
  drivePID(24.5);//knocks away red ring
  inert(180);
  drivePID(-7);
  mIntake.spin(fwd,100,pct);//puts ring on alliance stake
  /*
  sDoor.set(true);
  // Grab First Ring
  mIntake.spin(forward, 100, pct);
  drivePID(50);
  wait(600, msec);
  mIntake.stop();
  // Line up to Grab Goal
  inert(65);
  // Drive up to goal
  driveInchesClamp(-18, 50);
  // Grab Goal
  sClamp.set(false);
  wait(150, msec);
  // Turn to Center Ring Stack and Start Intaking
  mIntake.spin(forward, 100, pct);
  inert(-152);
  // Lift intake
  sintake.set(true);
  mIntake.spin(forward, 100, pct);
  // Lift Arm
  mLift.setVelocity(40, pct);
  mLift.spinFor(reverse, 260, degrees, false);
  // Drive over ring stack
  drivePID(22);
  driveInches(16, 60);
  // Drop intake on ring
  sintake.set(false);
  drivePID(3.5);
  drivePID(-3);
  mIntake.stop();
  // Line up to wall stake
  inert(-213);
  // Put arm on wall stake
  drivePID(11.5);
  // Push ring onto stake
  mLift.spinFor(forward, 230, degrees);
  mLift.stop(coast);
  wait(500, msec);
  // intake ring
  mIntake.spin(forward, 100, pct);
  // Drive back off wall stake
  drivePID(-15.8);
  // Drop goal out of the way
  inert(-115);
  sClamp.set(true);
  // Line up to grab second goal
  inert(-260);
  driveInchesClamp(-28.5, 50);
  inert(-115);
  drivePID(23);
  inert(45);

  drivePID(39);
  */
}
void halfAWP()
{
  // Brain.Screen.print(mMidLeft.temperature());
  sDoor.set(true);
  // Grab First Ring
  mIntake.spin(forward, 100, pct);
  drivePID(50);
  wait(300, msec);
  mIntake.stop();
  wait(100, msec);
  // Line up to Grab Goal
  inert(65);
  // Drive up to goal
  driveInchesClamp(-18, 50);
  // Grab Goal
  sClamp.set(false);
  wait(150, msec);
  // Turn to Center Ring Stack and Start Intaking
  mIntake.spin(forward, 60, pct);
  inert(-152);
  // Lift intake
  sintake.set(true);
  mIntake.spin(forward, 100, pct);
  // Lift Arm
  mLift.setVelocity(40, pct);
  mLift.spinFor(reverse, 260, degrees, false);
  // Drive over ring stack
  drivePID(22);
  driveInches(16, 60);
  // Drop intake on ring
  sintake.set(false);
  drivePID(3.5);
  drivePID(-3);
  mIntake.stop();
  // Line up to wall stake
  inert(-213);
  // Put arm on wall stake
  drivePID(11.5);
  // Push ring onto stake
  mLift.spinFor(forward, 230, degrees);
  mLift.stop(coast);
  wait(500, msec);
  // intake ring
  mIntake.spin(forward, 100, pct);
  // Drive back off wall stake
  drivePID(-15.8);
  // Drop goal out of the way
  inert(-115);
  sClamp.set(true);
  // Line up to grab second goal
  inert(-260);
  drivePID(5);
}

void adaptive()
{
  drivePID(10);
  
}

void autonomous(void)
{
  // auton = 25;
  // oldDrivePID(1000,100);
  // driveDeg(1000,1000,100);
  /*Brain.Screen.clearLine();
  Brain.Screen.print(mBackRight.position(degrees));*/
  // inert(90);
  switch (auton)
  {
  case 0:
    rightAuto();
    break;
  case 1:
    blueRightAuto(-1); // redLeftAuto is blueRightAuto but inversed
    break;
  case 2:
    blueRightAuto(1);
    break;
  case 3:
    skillsAuto();
    break;
  case 4:
    AWP();
    break;
  case 5:
    halfAWP();
    break;
  case 6:
    adaptive();
    break;
  case 7:
    blueMidAuto();
    break;
  }
  // inert(90);
  mBackLeft.stop();
  mBackRight.stop();
  mFrontLeft.stop();
  mFrontRight.stop();
  mMidLeft.stop();
  mMidRight.stop();
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
bool clamp = false;
bool door = false;
bool upInt = false;
bool macro = false;
int count = 0;
int armCount = 0;
int armPos = 1;
bool definedVar = false;
bool running = false;
const double c = 31.62278;
/* temp evan code for calculating c
const double logDriveMag = 1.75;  // sets magnitude for log drive
const double logDriveAligner = floor(pow(100,logDriveMag) * 100)/10000; // 100 pwr at 100 pct control
*/
bool pidRunning = false;

const double d = 1.75;

double targetDeg;
double kP = 80;
double kI = 0;
double kD = 1;
int goalMet;              // Flag to track if the goal is met
double currentDelta;      // Error between target and current position
double P, I, D, totalPID; // PID terms
double pollingRate;       // Polling rate in ms // Target position in degrees

double previousDelta; // Initialize previous error as target
double integralSum;   // Cumulative error for integral term

double targetDegInpRot;
double targetDegInp;

bool skip = false;
double prevRot = 0;
void usercontrol(void)
{
  mBackLeft.stop(coast);
  mBackRight.stop(coast);
  mFrontLeft.stop(coast);
  mFrontRight.stop(coast);
  mMidLeft.stop(coast);
  mMidRight.stop(coast);
  Rotation.resetPosition();
  mLift.setPosition(0,deg);
  // User control code here, inside the loop
  while (1)
  {

    double right = Controller1.Axis2.position(percent);
    double left = Controller1.Axis3.position(percent);

    if (left >= 0)
    {
      left = pow(left, d) / c;
    }
    else if (left < 0)
    {
      left = -(pow(-left, d) / c);
    }

    if (right >= 0)
    {
      right = pow(right, d) / c;
    }
    else if (right < 0)
    {
      right = -(pow(-right, d) / c);
    }

    mFrontRight.spin(forward, right, pct);
    mBackRight.spin(forward, right, pct);
    mBackLeft.spin(forward, left, pct);
    mFrontLeft.spin(forward, left, pct);
    mMidLeft.spin(forward, left, pct);
    mMidRight.spin(forward, right, pct);

    if (Controller1.ButtonY.pressing())
    { // enables PID Tuning mode *** DISABLE DURING COMPS
      tunePID();
      std::cout << "enabling tunePID." << std::endl;
    }

    if (Controller1.ButtonX.pressing())
    { // toggle for the macro
      if (macro == false)
      {
        macro = true;
      }
      else if (macro == true)
      {
        macro = false;
        count = 0;
      }
      wait(240, msec);
    }
    if (macro == true)
    { // uses count variable to time length
      mIntake.spin(fwd, 100, pct);
      if (ringCheck.objectDistance(inches) <= 1.5)
      {
        count += 1;
      }
      if (count >= 4)
      { // sees the ring for 80 millseconds, it starts reversing
        mIntake.spin(reverse, 100, pct);
        count += 1;
      }
      if (count >= 150)
      { // after another half a second it resets to original value
        macro = false;
        count = 0;
      }
    }
    if (Controller1.ButtonR1.pressing())
    {
      mIntake.spin(fwd, 100, pct);
    }
    else if (Controller1.ButtonR2.pressing())
    {
      mIntake.spin(reverse, 100, pct);
    }
    else if (macro == false)
    {
      mIntake.stop();
    }

    if (Controller1.ButtonB.pressing())
    {
      if (clamp == false)
      {
        clamp = true;
      }
      else if (clamp == true)
      {
        clamp = false;
      }
      sClamp.set(clamp);
      vex ::wait(240, msec);
    }
    //cannot call the same pos twice
    if((Controller1.ButtonL1.pressing()||Controller2.ButtonL1.pressing()) && pidRunning==false && armPos!=1){ 
      pidRunning = true; //starts the PID running
      armPos =1; //sets the position of the arm, stores in variable 
      targetDegInpRot = 0;  //rotation sensor value for PID
      targetDegInp = 0; //tune according to motor encoder values
    }
    else if((Controller1.ButtonL2.pressing()||Controller2.ButtonL2.pressing()) && pidRunning==false  && armPos!=2){//cannot call the same pos twice
      pidRunning = true;
      armPos=2;
      targetDegInpRot = 25; //rotation sensor value
      targetDegInp = 65; //tune according to motor encoder values, could be negative idk
    }
    else if((Controller1.ButtonLeft.pressing()||Controller2.ButtonLeft.pressing()) && pidRunning==false  && armPos!=3){
      pidRunning = true;
      armPos=3;
      targetDegInpRot = 131.5;
      targetDegInp = 220; //tune according to motor encoder values, could be negative idk
    }


/*
    if(Controller2.ButtonR1.pressing()){
      mLift.spin(fwd,70,pct);
    }
    else if(Controller2.ButtonR2.pressing()){
      mLift.spin(reverse,70,pct);
    }
    else{
      mLift.stop(hold);
    }
*/
    if(Controller2.ButtonB.pressing()){
      mLift.setPosition(0,deg);
      mLift.stop(hold);
    }
    else if(Controller2.ButtonA.pressing()){
      mLift.setPosition(65,deg);
       mLift.stop(hold);
    }
    else if(Controller2.ButtonX.pressing()){
      mLift.setPosition(220,deg);
      mLift.stop(hold);
    }

    //This code uses the rotation sensor to turn the motor
    if(pidRunning==true){//the code runs completely because of this variable
      if(definedVar==false){// The variable is what makes it so the values are all only defined once
        targetDeg = targetDegInpRot ;
        kP = 1500;
        kI = 0;
        kD = 0.01;
        goalMet = 0;                      // Flag to track if the goal is met
        currentDelta;                  // Error between target and current position
        P = 0, I = 0, D = 0; // PID terms            // Polling rate in ms // Target position in degrees
        prevRot = Rotation.angle(deg);
        
        previousDelta = targetDeg; // Initialize previous error as target
        integralSum = 0;  

        definedVar=true;
      }
       if (goalMet <= 1 && definedVar==true)
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

          if(armCount%15==0 && armCount>=50){
            if(fabs(prevRot-currentPosition)<30){
              skip==true;
            }
            else{
              prevRot = currentPosition;
            }
          }
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
          if (fabs(currentDelta) < 2 || skip==true)
          {
            goalMet++;
            pidRunning=false;
            definedVar=false;
          }
          // Update the previous error for the next loop
          previousDelta = currentDelta;
          armCount+=1;
          // Wait for the polling rate before next iteration
          // changed to nothing
        }
        // Stop the motors once goal is met
    }
    else if(Controller1.ButtonDown.pressing()){//code for correcting inacuracies in the code manually
      mLift.spin(reverse,70,pct);
      mLift.setPosition(0,deg);
      armPos = 1;
    }
    else if(Controller2.ButtonR1.pressing()){
      mLift.spin(fwd,15,pct);
    }
    else if(Controller2.ButtonR2.pressing()){
      mLift.spin(reverse,15,pct);
    }
    else{
      mLift.stop(hold);
      mLift2.stop(hold);
    }
    //This is the motor encoder version <------------------------------------
    /*
    if(pidRunning==true){
       if(definedVar==false){
        
        targetDeg = targetDegInp ;
        kP = 1000;
        kI = 0;
        kD = .5;
        goalMet = 0;                      // Flag to track if the goal is met
        currentDelta;                  // Error between target and current position
        P = 0, I = 0, D = 0 ;// PID terms            // Polling rate in ms // Target position in degrees

        previousDelta = targetDegInp; // Initialize previous error as target
        integralSum = 0;  

        definedVar=true;
      }
       if (goalMet <= 1 && definedVar==true)
        {
          // Main PID loop; runs until target is reached
          // Read motor position (you can average left and right motor values for straight driving)
          double currentPosition = mLift.position(deg);

          double error = targetDeg - currentPosition;
          // Calculate the current error
          currentDelta = error;

          // Proportional: Larger error results in larger response
          P = (kP / 1000) * currentDelta;

          // Integral: Sum of all errors helps correct for small errors over time
          integralSum += currentDelta;
          I = kI * integralSum;

          // Derivative: React to the rate of error change
          D = kD * (currentDelta - previousDelta) / 20;

          // Calculate total PID response
          totalPID = P + I + D;

          // Use totalPID to move motors proportionally
          mLift.spin(forward, totalPID, percent);

          // Check if the error is small enough to stop
          if (fabs(currentDelta) < 3)
          {
            goalMet++;
            pidRunning=false;
            definedVar=false;
          }
          // Update the previous error for the next loop
          previousDelta = currentDelta;

          // Wait for the polling rate before next iteration
          // changed to nothing
        }
    }
    else if(Controller1.ButtonDown.pressing()){
      mLift.spin(reverse,70,pct);
      mLift.setPosition(0,deg);
      armPos = 1;
    }
    else if(Controller2.ButtonR1.pressing()){
      mLift.spin(fwd,15,pct);
    }
    else if(Controller2.ButtonR2.pressing()){
      mLift.spin(reverse,15,pct);
    }
    else{
      mLift.stop(hold);
    }*/
    //THIS IS THE INCORRECT ARM CODE
      /*  if ((Controller1.ButtonL1.pressing())||(armPos==1&&running==true))
        {
          if (definedVar == false && running == false)
          {
            double targetDeg = 355;
            int goalMet = 0;                       // Flag to track if the goal is met
            double currentDelta;                  // Error between target and current position
            double P = 0, I = 0, D = 0, totalPID; // PID terms
            double pollingRate = 20;              // Polling rate in ms // Target position in degrees

            double previousDelta = targetDeg; // Initialize previous error as target
            double integralSum = 0;           // Cumulative error for integral term

            // Reset motor encoder value to 0
            mLift.setPosition(0, degrees);

            definedVar = true;
            running = true;
            armPos = 1;
          }
          if (goalMet <= 1 && running == true && armPos == 1)
          {
            // Main PID loop; runs until target is reached
            // Read motor position (you can average left and right motor values for straight driving)
            double currentPosition = Rotation.angle(deg);

            // Calculate the current error
            currentDelta = targetDeg - currentPosition;

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
            mLift.spin(forward, totalPID, percent);

            // Check if the error is small enough to stop
            if (fabs(currentDelta) < 30)
            {
              goalMet++;
            }
            // Update the previous error for the next loop
            previousDelta = currentDelta;

            // Wait for the polling rate before next iteration
            // changed to nothing
          }
          // Stop the motors once goal is met
          mLift.stop(hold);
          running = false;
          definedVar = false; // implement counting feature to avoid having waits in driver control
        }
        else if ((Controller1.ButtonL2.pressing()) || (armPos==2 &&running==true))
        {
          if (definedVar == false && running == false)
          {
            double targetDeg = 332;
            int goalMet = 0;                      // Flag to track if the goal is met
            double currentDelta;                  // Error between target and current position
            double P = 0, I = 0, D = 0, totalPID; // PID terms
            double pollingRate = 20;              // Polling rate in ms // Target position in degrees

            double previousDelta = targetDeg; // Initialize previous error as target
            double integralSum = 0;           // Cumulative error for integral term

            // Reset motor encoder value to 0
            mLift.setPosition(0, degrees);

            definedVar = true;
            running = true;
            armPos = 2;
          }

          if (goalMet <= 1 && running == true && armPos == 2)
          {
            // Main PID loop; runs until target is reached
            // Read motor position (you can average left and right motor values for straight driving)
            double currentPosition = Rotation.angle(deg);

            // Calculate the current error
            currentDelta = targetDeg - currentPosition;

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
            mLift.spin(forward, totalPID, percent);

            // Check if the error is small enough to stop
            if (fabs(currentDelta) < 30)
            {
              goalMet++;
            }
            // Update the previous error for the next loop
            previousDelta = currentDelta;

            // Wait for the polling rate before next iteration
            // changed to nothing
          }
          // Stop the motors once goal is met
          mLift.stop(hold);
          running = false;
          definedVar = false; // implement counting feature to avoid having waits in driver control
        }
        else if ((Controller1.ButtonL.pressing()) || (armPos==3 &&running==true))
        {
          if (definedVar == false && running == false)
          {
            double targetDeg = 232;
            int goalMet = 0;                      // Flag to track if the goal is met
            double currentDelta;                  // Error between target and current position
            double P = 0, I = 0, D = 0, totalPID; // PID terms
            double pollingRate = 20;              // Polling rate in ms // Target position in degrees

            double previousDelta = targetDeg; // Initialize previous error as target
            double integralSum = 0;           // Cumulative error for integral term

            // Reset motor encoder value to 0
            mLift.setPosition(0, degrees);

            definedVar = true;
            running = true;
            armPos = 3;
          }

          if (goalMet <= 1 && running == true && armPos == 3)
          {
            // Main PID loop; runs until target is reached
            // Read motor position (you can average left and right motor values for straight driving)
            double currentPosition = Rotation.angle(deg);

            // Calculate the current error
            currentDelta = targetDeg - currentPosition;

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
            mLift.spin(forward, totalPID, percent);

            // Check if the error is small enough to stop
            if (fabs(currentDelta) < 30)
            {
              goalMet++;
            }
            // Update the previous error for the next loop
            previousDelta = currentDelta;

            // Wait for the polling rate before next iteration
            // changed to nothing
          }
          // Stop the motors once goal is met
          mLift.stop(hold);
          running = false;
          definedVar = false; // implement counting feature to avoid having waits in driver control
        }
        else
        {
          mLift.stop(hold);
        }*/
    

    /*if (Controller1.ButtonDown.pressing())
    {
      if (door == false)
      {
        door = true;
      }
      else if (door == true)
      {
        door = false;
      }
      sDoor.set(door);
      vex ::wait(240, msec);
    }*/

    if (Controller1.ButtonUp.pressing())
    {
      if (upInt == false)
      {
        upInt = true;
      }
      else if (upInt == true)
      {
        upInt = false;
      }
      sintake.set(upInt);
      vex ::wait(240, msec);
    }

    /*Controller1.Screen.clearLine();
    Controller1.Screen.print(Inertial.rotation());
    Controller1.Screen.print(" , ");
    Controller1.Screen.print(mBackRight.position(degrees));*/
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    vex ::wait(20, msec);
    // Brain.Screen.clearLine(); // Sleep the task for a short amount of time to
    //  prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    vex ::wait(100, msec);
  }
}

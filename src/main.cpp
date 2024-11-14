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

#include "main.h"
#include "devices.h"
#include "movement.h"
#include "auton.h"
#include "vex.h"
#include <iostream>
#include <string>
using namespace vex;

void printCenter(std::string input)
{
  // Screen Size (x,y) = (480, 240)
  int width = Brain.Screen.getStringWidth(input.c_str());
  int xOffset = 240 - width/2;
  if (width <= 480){
    Brain.Screen.clearLine();
    Brain.Screen.printAt(xOffset, 120, input.c_str());
  }
}

int autonSelection = 0;
int autonCount = 7;
void autonSelect()
{
  autonSelection = (autonSelection + 1)%(autonCount + 1); // loops 1 through the auton count
  switch (autonSelection)
  {
  case 0:
    Brain.Screen.clearLine(); 
    printCenter("Right side");
    break;
  case 1:
    Brain.Screen.clearLine();
    printCenter("Red Left Auto");
    break;
  case 2:
    Brain.Screen.clearLine();
    printCenter("Blue Right Auto");
    break;
  case 3:
    Brain.Screen.clearLine();
    printCenter("Skills");
    break;
  case 4:
    Brain.Screen.clearLine();
    printCenter("AWP");
    break;
  case 5:
    Brain.Screen.clearLine();
    printCenter("Half AWP");
    break;
  case 6:
    Brain.Screen.clearLine();
    printCenter("adaptive");
    break;
  case 7:
    Brain.Screen.clearLine();
    printCenter("blue Mid Auto");
    break;
  }
}

void pre_auton(void)
{
  sClamp.set(true);
  Inertial.calibrate();
  while(Inertial.isCalibrating()){ // temporarily freezes robot during sensor calibration
    Brain.Screen.clearScreen();
    printCenter("Inertial Sensor Calibrating");
    wait(50, msec);
  }
  Brain.Screen.clearScreen();
  vex::thread odom(odometry);
  vex::thread odomData(odomDataCollection);
  Brain.Screen.pressed(autonSelect);
  
  vex ::wait(4, sec);
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

void autonomous(void)
{
  // auton = 25;
  // oldDrivePID(1000,100);
  // driveDeg(1000,1000,100);
  /*Brain.Screen.clearLine();
  Brain.Screen.print(mBackRight.position(degrees));*/
  // inert(90);
  switch (autonSelection)
  {
  case 0:
    rightAuto();
    break;
  case 1:
    halfAWP(-1); // redLeftAuto is blueRightAuto but inversed
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
    halfAWP(1);
    break;
  case 6:
    blueMidAuto(-1);
    break;
  case 7:
    blueMidAuto(1);
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

event Event = event(setArmBottom);
event Event2 = event(setArmMid);
event Event3 = event(setArmTop);
vex::thread arm();
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
    {
      macro = !macro;  // Toggle macro state
      if (!macro) count = 0;  // Reset count when turning off
      wait(240, msec);
    }
    if (macro) {
      if (count >= 150) {  // After ~0.5s of reversing
          macro = false;
         count = 0;
      }
      else if (count >= 4) {  // After 80ms of ring detection
          mIntake.spin(reverse, 100, pct);
          count++;
      }
      else {  // Initial forward motion
        mIntake.spin(fwd, 100, pct);
          if (ringCheck.objectDistance(inches) <= 1.5) {
            count++;
          }
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

    if (Controller1.ButtonB.pressing()) {
    clamp = !clamp;  // Toggle clamp state
    sClamp.set(clamp);
    wait(240, msec);
    }

    //cannot call the same pos twice
    if((Controller1.ButtonL1.pressing()||Controller2.ButtonL1.pressing()) && pidRunning==false && armPos!=1){ 
      pidRunning = true; //starts the PID running
      armPos =1; //sets the position of the arm, stores in variable 
      targetDegInpRot = 0;  //rotation sensor value for PID
      targetDegInp = 0; //tune according to motor encoder values
      Event(setArmBottom);
    }
    else if((Controller1.ButtonL2.pressing()||Controller2.ButtonL2.pressing()) && pidRunning==false  && armPos!=2){//cannot call the same pos twice
      pidRunning = true;
      armPos=2;
      targetDegInpRot = 25; //rotation sensor value
      targetDegInp = 65; //tune according to motor encoder values, could be negative idk
       Event2(setArmMid);
    }
    else if((Controller1.ButtonLeft.pressing()||Controller2.ButtonLeft.pressing()) && pidRunning==false  && armPos!=3){
      pidRunning = true;
      armPos=3;
      targetDegInpRot = 131.5;
      targetDegInp = 220;
      Event3(setArmTop);
       //tune according to motor encoder values, could be negative idk
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

//THIS IS THE ARM CODE WITHOUT MULTITHREADING

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
              //skip=true; The arm is working fine without this, but can add back if needed
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
    //else if(Controller1.ButtonDown.pressing()){//code for correcting inacuracies in the code manually
    //  mLift.spin(reverse,70,pct);
    //  mLift.setPosition(0,deg);
   //   armPos = 1;
    //}
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

    Event.broadcast();
    Event2.broadcast();
    Event3.broadcast();
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

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       4478B                                                     */
/*    Created:      Sat Nov 16 2024                                           */
/*    Description:  Competition functions                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "main.h"
#include "devices.h"
#include "movement.h"
#include "auton.h"
#include "vex.h"
#include <iostream>
#include <string>
using namespace vex;

void pre_auton(void)
{
  sClamp.set(true);
  Inertial.calibrate();
  while(Inertial.isCalibrating()){ // pauses while calibrating IMU
    Brain.Screen.clearScreen();
    printCenter("Inertial Sensor Calibrating");
    wait(50, msec);
  }
  Controller1.rumble("-"); // rumble indicates finished calibration
  Brain.Screen.clearScreen();

  //vex::thread odom(odometry); // after calibrated, can start odometry
  //vex::thread odomData(odomDataCollection);

  AutonSelector& selector = getAutonSelector();
    selector.displayCurrentSelection();
    Brain.Screen.pressed([]() {
        getAutonSelector().nextSelection();
    });
    
  vex ::wait(4, sec);
}

void autonomous(void)
{
  // Calibrates motor encoders to reduce drift
  allMotors.setPosition(0,deg);
  Rotation.resetPosition();
  armMotors.setPosition(0,deg);

  // Performs selected autonomous route
  getAutonSelector().runSelectedAuton();
  
  // Stops motors to prevent crossing field
  allMotors.stop();
}

bool clamp = false;
bool door = false;
bool upInt = false;
bool macro = false;
int count = 0;
int armCount = 0;
int armPos = 1;
bool definedVar = false;

bool running = false;
bool pidRunning = false;

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

double prevRot = 0;

/*event Event = event(setArmBottom);
event Event2 = event(setArmMid);
event Event3 = event(setArmTop);
vex::thread arm();*/

void handleTuner(){
  if (Controller1.ButtonY.pressing())
    {
      std::cout << "enabling tunePID." << std::endl;
      tunePID();
    }
}

void handleMacro(){
  if (Controller1.ButtonX.pressing())
    {
      macro = !macro;  // Toggle macro state
      if (!macro) count = 0;  // Reset count when turning off
      // Debounce delay to prevent multiple toggles
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
}

void handleIntake(){
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

  if (Controller1.ButtonUp.pressing()) {
    // Toggle intake to opposite of current state
    sIntake.set(!sIntake.value());
    // Debounce delay to prevent multiple toggles
    vex::wait(240, msec);
  } 
}

void handleClamp(){
  if (Controller1.ButtonB.pressing()) {
    clamp = !clamp;  // Toggle clamp state
    sClamp.set(clamp);
    // Debounce delay to prevent multiple toggles
    wait(240, msec);
    }
}

void handleArm(){

    //cannot call the same pos twice
    if((Controller1.ButtonL1.pressing()||Controller2.ButtonL1.pressing()) && pidRunning==false && armPos!=1){ 
      pidRunning = true; //starts the PID running
      armPos =1; //sets the position of the arm, stores in variable 
      targetDegInpRot = 0;  //rotation sensor value for PID
      targetDegInp = 0; //tune according to motor encoder values
      //Event(setArmBottom);
    }
    else if((Controller1.ButtonL2.pressing()||Controller2.ButtonL2.pressing()) && pidRunning==false  && armPos!=2){//cannot call the same pos twice
      pidRunning = true;
      armPos=2;
      targetDegInpRot = 25; //rotation sensor value
      targetDegInp = 65; //tune according to motor encoder values, could be negative idk
      //Event2(setArmMid);
    }
    else if((Controller1.ButtonLeft.pressing()||Controller2.ButtonLeft.pressing()) && pidRunning==false  && armPos!=3){
      pidRunning = true;
      armPos=3;
      targetDegInpRot = 131.5;
      targetDegInp = 220;
      //Event3(setArmTop);
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


// this is to reset the arms' encoder if it is offset
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
//THIS IS THE ARM CODE WITHOUT MULTITHREADING
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

          prevRot = currentPosition;

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

/*
    if(armPos==1){

         Event.broadcast();
    }
    else if(armPos==2){
      Event2.broadcast();
    }
    else if(armPos==3){
      Event3.broadcast();
    }*/

}

void usercontrol(void)
{
  allMotors.stop(coast);
  Rotation.resetPosition();
  armMotors.setPosition(0,deg);
  // User control code here, inside the loop
  while (1)
  {

    leftMotors.spin(forward, logDriveJoystick(Controller1.Axis3.position(percent)), pct);
    rightMotors.spin(forward, logDriveJoystick(Controller1.Axis2.position(percent)), pct);

    //handleTuner(); // *** DISABLE DURING COMPS ***
    handleMacro();
    handleIntake();
    handleClamp();
    handleArm();
    
    vex ::wait(20, msec);
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

void printCenter(const char* input)
{
  // Screen Size (x,y) = (480, 240)
  int width = Brain.Screen.getStringWidth(input);
  int xOffset = 240 - width/2;
  if (width <= 480){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(xOffset, 120, input);
  }
}
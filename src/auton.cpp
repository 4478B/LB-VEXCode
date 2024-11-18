/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       auton.cpp                                                 */
/*    Author:       4478B                                                     */
/*    Created:      Sat Nov 16 2024                                           */
/*    Description:  auton selector and auton routes                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "auton.h"
#include "devices.h"
#include "movement.h"
#include "vex.h"
#include "main.h"
#include <iostream>
using namespace vex;

/*
 * Framework for the autonomous selector.
 * Add/remove routines by editing the "routines" array below.
 */

// Full definition of AutonRoutine struct
struct AutonRoutine {
    const char* displayName;
    std::function<void(int)> routine;
    int multiplier;
};

// Define the routines array
static const AutonRoutine ROUTINES[] = {
    {"Right side", rightAuto, 1},      // Route 1
    {"Red Left Auto", halfAWP, -1},    // Route 2
    {"Blue Right Auto", blueRightAuto, 1}, // Route 3
    {"Skills", skillsAuto, 1},         // Route 4
    {"AWP", AWP, 1},                   // Route 5
    {"Blue 3 Ring, Ring side", halfAWP, 1},  // Route 6
    {"Blue Left Auto", rightAuto, -1}, // Route 7
    {"Blue Mid Auto", blueMidAuto, 1}  // Route 8
};

// Create static instance
static AutonSelector instance;

// Constructor implementation
AutonSelector::AutonSelector() : 
    routines(ROUTINES),
    currentSelection(5),  // Start with AWP (Route 5)
    routineCount(sizeof(ROUTINES) / sizeof(ROUTINES[0]))
{}

void AutonSelector::nextSelection() {
    currentSelection = (currentSelection % routineCount) + 1;
    displayCurrentSelection();
}

void AutonSelector::displayCurrentSelection() {
    Brain.Screen.clearLine();
    printCenter(routines[currentSelection - 1].displayName);
}

void AutonSelector::runSelectedAuton() {
    const AutonRoutine& selected = routines[currentSelection - 1];
    selected.routine(selected.multiplier);
}

int AutonSelector::getCurrentSelection() const {
    return currentSelection;
}

// Global instance getter implementation
AutonSelector& getAutonSelector() {
    return instance;
}

//=============================================================================

/* Autonomous routines */
void skillsAuto(int i)
{
  driveInchesClamp(-200, 40);  // VALUE NEEDS TO BE TWEAKED
  mIntake.spin(fwd, 100, pct);
  wait(700,msec);
  inert(-90);
  driveInches(35,50);
  inert(135);
  sClamp.set(true);
  driveInches(-27,40);
  drivePID(25);
  inert(90);
  drivePID(130);
  inert(45);
  drivePID(30);
  drivePID(-30);
  inert(180);
  drivePID(120);
  inert(-90);
  drivePID(30);
  //drivePID(-30);
  //inert(270);
  /*drivePID(108);
  inert(225);
  drivePID(30);
  drivePID(-40);
  drivePID(30);
  inert(95);*/
  driveInches(400,70);
  driveInches(-30,50);
}
void blueRightAuto(int i) // i is if its inversed or not (1 = regular, -1 = inversed)
{
  // this auto is intended to score 5 rings in one goal and score one on a side stake
  // Grab Goal
  driveInchesClamp(-1320, 50);  // VALUE NEEDS TO BE TWEAKED
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
  sIntake.set(true);
  drivePID(40);
  sIntake.set(false);
  drivePID(-15);
}
void blueMidAuto(int i)
{
  // Starting position: In blue right corner, facing mobile goal
  drivePID(-27); 
  driveInchesClamp(-300, 30); // VALUE NEEDS TO BE TWEAKED
  inert(-104*i);
  wait(500, msec);
  mIntake.spin(forward, 100, pct);
  drivePID(36);
  wait(500, msec); 
  drivePID(-27);
  wait(500,msec);
  inert(-55*i);
  drivePID(22);
  wait(500,msec);
  drivePID(-10);
  drivePID(-30);
  sClamp.set(true);
  drivePID(14);
  inert(35*i);
  mIntake.stop();
  drivePID(30);
  inert(125*i);
  drivePID(60);     
  inert(0);
  driveInchesClamp(-1400,80);  // VALUE NEEDS TO BE TWEAKED
  wait(300,msec);
  inert(125*i);
  mIntake.spin(fwd, 100,pct);
  drivePID(30);
}
void redleftAuto(int i)
{
  // this auto is intended to score 5 rings
  // in one goal and score one on a side stake
  // Grab Goal
  driveInchesClamp(-1100, 80);  // VALUE NEEDS TO BE TWEAKED
  // Turn on intake
  mIntake.spin(forward, 100, pct);
  // Turn towards mid ring stack
  inert(120);

  Inertial.setHeading(140,deg);
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
  sIntake.set(true);
  mIntake.spin(fwd, 60, pct);
  drivePID(68);
  sIntake.set(false);
  // put ring on alliance stake after
  // turning and lift life
  driveInches(-10, 50); // VALUE NEEDS TO BE TWEAKED
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
void rightAuto(int i) // inverse is blue left
{
  driveInchesClamp(-28,50);
  mIntake.spin(fwd,100,pct);
  wait(400, msec);
  inert(-90*i);
  drivePID(-5);
  sClamp.set(true);
  drivePID(35);
  wait(300,msec);
  mIntake.stop();
  inert(0*i);
  driveInchesClamp(-17,30);
  drivePID(24);
  mIntake.spin(fwd,100,pct);
  wait(300, msec);
  inert(100*i);

  setArmTop();
  driveInches(60,40);
















}
void AWP(int i)
{ // starts out the same as rightAuto but then goes full field
  driveInchesClamp(-28,50);
  mIntake.spin(fwd,100,pct);
  wait(400, msec);
  inert(-90);
  drivePID(-5);
  sClamp.set(true);
  wait(50,msec);
  drivePID(34);
  wait(300,msec);
  mIntake.stop();
  inert(0);
  driveInchesClamp(-17,30);
  mIntake.spin(fwd, 100, pct);
  drivePID(9);
  wait(300,msec);
  inert(94);
  mIntake.stop();
  sClamp.set(true);










  drivePID(75);
  inert(225);
  driveInchesClamp(-20,50);
  
 

  /* THIS OLD AWP CODE WAS REPLACED BEFORE DANIEL HAND COMP
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
  sIntake.set(true);
  drivePID(57);
  sIntake.set(false);//grabs ring by alliance stake
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
  sIntake.set(true);
  mIntake.spin(forward, 100, pct);
  // Lift Arm
  mLift.setVelocity(40, pct);
  mLift.spinFor(reverse, 260, degrees, false);
  // Drive over ring stack
  drivePID(22);
  driveInches(16, 60);
  // Drop intake on ring
  sIntake.set(false);
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
void halfAWP(int i) // this is daniel hand redleft and blueright
{
  drivePID(-27); 
  driveInchesClamp(-300, 30);  // VALUE NEEDS TO BE TWEAKED
  wait(500,msec);
  mIntake.spin(forward, 100, pct);
  /*
  inert(-108*i);
  wait(500, msec);
  
  drivePID(36);
  wait(500, msec); 
  drivePID(-27);
  wait(500,msec);
  */
  inert(-55*i);
  drivePID(27);//+3; nvm
  wait(500,msec);
  drivePID(-4);

  inert(-145*i);
  drivePID(20);
  wait(500,msec);
  drivePID(-20);
  inert(-55*i);

  drivePID(-11);//-3; nvm
  drivePID(-30);
  setArmTop();   
  inert(-145*i);
  drivePID(7);
}
void adaptive(int i)
{
  drivePID(-50);
  
}
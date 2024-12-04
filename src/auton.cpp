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
struct AutonRoutine
{
  const char *displayName;
  std::function<void(int)> routine;
  int multiplier;
};

// Define the routines array
static const AutonRoutine ROUTINES[] = {
    {"Red Goal Side Two Top Rings", rightAuto, 1},      // Route 1
    {"Red 3 Ring, Ring side", halfAWP, -1},    // Route 2
    {"Elim Blue 4 Ring, Ring Side ", AWP, 1}, // Route 3
    {"Skills", skillsAuto, 1},         // Route 4
    {"Elim Red 4 Ring, Ring Side ", AWP, -1},                   // Route 5
    {"Blue 3 Ring, Ring side", halfAWP, 1},  // Route 6
    {"Blue Goal Side Two Top Rings", rightAuto, -1}, // Route 7
    {"Red AWP Goal Side, Full Field, 3 Ring", blueMidAuto, 1}  // Route 8
};

// Create static instance
static AutonSelector instance;

// Constructor implementation
AutonSelector::AutonSelector() : routines(ROUTINES),
                                 currentSelection(5), // Start with AWP (Route 5)
                                 routineCount(sizeof(ROUTINES) / sizeof(ROUTINES[0]))
{
}

void AutonSelector::nextSelection()
{
  currentSelection = (currentSelection % routineCount) + 1;
  displayCurrentSelection();
}

void AutonSelector::displayCurrentSelection()
{
  Brain.Screen.clearLine();
  printCenter(routines[currentSelection - 1].displayName);
}

void AutonSelector::runSelectedAuton()
{
  const AutonRoutine &selected = routines[currentSelection - 1];
  selected.routine(selected.multiplier);
}

int AutonSelector::getCurrentSelection() const
{
  return currentSelection;
}

// Global instance getter implementation
AutonSelector &getAutonSelector()
{
  return instance;
}

//=============================================================================

/* Autonomous routines */
void skillsAuto(int i)
{
  driveInchesClamp(-7, 40); // VALUE NEEDS TO BE TWEAKED
  mIntake.spin(fwd, 100, pct);
  wait(700, msec);
  inert(-90);
  driveInches(35, 50);
  inert(135);
  sClamp.set(true);
  driveInches(-27, 40);
  drivePID(25);
  inert(90);
  drivePID(130);
  inert(45);
  drivePID(30);
  drivePID(-30);
  inert(180);
  drivePID(120);
  inert(-100);
  drivePID(30);
  // drivePID(-30);
  // inert(270);
  /*drivePID(108);
  inert(225);
  drivePID(30);
  drivePID(-40);
  drivePID(30);
  inert(95);*/
  driveInches(136, 70);
  driveInches(-15, 50);
  inert(-240);
  driveInches(140, 70);
}
void blueRightAuto(int i) // i is if its inversed or not (1 = regular, -1 = inversed)
{
  // this auto is intended to score 5 rings in one goal and score one on a side stake
  // Grab Goal
  driveInchesClamp(-1320, 50); // VALUE NEEDS TO BE TWEAKED
  // Turn towards mid ring stack
  inert(i * -140);
  mIntake.spin(forward, 100, pct); // Turn on intake
  // Pick up bottom ring
  drivePID(27);
  wait(200, msec);
  // Back up to avoid intaking second ring
  drivePID(-7);
  // Turn towards Alliance side one stack
  inert(i * -42);
  // waiting for ring to intake
  wait(300, msec);
  // Pick up red side one stack
  drivePID(17);
  wait(500, msec);
  inert(i * 196); // turn towards other mid ring

  drivePID(18.5);
  inert(i * 180);
  drivePID(-56);
  wait(300, msec);
  inert(i * -45);
  sIntake.set(true);
  drivePID(40);
  sIntake.set(false);
  drivePID(-15);
}
void blueMidAuto(int i)
{
   driveInchesClamp(-35, 50);
  mIntake.spin(fwd, 100, pct);
  wait(400, msec);
  inert(-63 * i);
  drivePID(-5);
  sClamp.set(true);
    driveInches(30, 80);

  wait(300, msec);
  mIntake.stop();
  inert(29 * i);
  driveInchesClamp(-23, 40);
  drivePID(26);
  mIntake.spin(fwd, 100, pct);
  wait(300, msec);

  drivePID(15);

  inert(100 * i);


  driveInches(60,70); //was 60 before
   mIntake.stop();
wait(200,msec);
   inert(158 * i);
   driveInches(55,70); //was 60 before
   mIntake.spin(fwd, 100, pct);
   driveInches(25,50);
  
 /* // Starting position: In blue right corner, facing mobile goal
  drivePID(-27);
  driveInchesClamp(-300, 30); // VALUE NEEDS TO BE TWEAKED
  inert(-104 * i);
  wait(500, msec);
  mIntake.spin(forward, 100, pct);
  drivePID(36);
  wait(500, msec);
  drivePID(-27);
  wait(500, msec);
  inert(-55 * i);
  drivePID(22);
  wait(500, msec);
  drivePID(-10);
  drivePID(-30);
  sClamp.set(true);
  drivePID(14);
  inert(35 * i);
  mIntake.stop();
  drivePID(30);
  inert(125 * i);
  drivePID(60);
  inert(0);
  driveInchesClamp(-1400, 80); // VALUE NEEDS TO BE TWEAKED
  wait(300, msec);
  inert(125 * i);
  mIntake.spin(fwd, 100, pct);
  drivePID(30);*/
}
void redleftAuto(int i)
{
  // this auto is intended to score 5 rings
  // in one goal and score one on a side stake
  // Grab Goal
  driveInchesClamp(-1100, 80); // VALUE NEEDS TO BE TWEAKED
  // Turn on intake
  mIntake.spin(forward, 100, pct);
  // Turn towards mid ring stack
  inert(120);

  Inertial.setHeading(140, deg);
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
  driveInchesClamp(-28, 50);
  mIntake.spin(fwd, 100, pct);
  wait(400, msec);
  inert(-135 * i);
  drivePID(-10);
  sClamp.set(true);
  drivePID(10);
  inert(-90 *i);
  drivePID(31);
  wait(300, msec);
  mIntake.stop();
  inert(0 * i);
  driveInchesClamp(-26, 30);
  drivePID(24);
  mIntake.spin(fwd, 100, pct);
  wait(300, msec);

  drivePID(15);

  inert(120 * i);

  setArmTop();
  driveInches(60,40); //was 60 before
}
void AWP(int i)
{
  drivePID(-27);
  driveInchesClamp(-7, 30); // VALUE NEEDS TO BE TWEAKED
  wait(500, msec);
  mIntake.spin(forward, 100, pct);
  /*
  inert(-108*i);
  wait(500, msec);

  drivePID(36);
  wait(500, msec);
  drivePID(-27);
  wait(500,msec);
  */
  inert(-55 * i);
  drivePID(27); //+3; nvm
  wait(500, msec);
  drivePID(-4);

  inert(-145 * i);
  driveInches(12.9,30);
  wait(500, msec);

  drivePID(-25);
  inert(-123 * i);
  drivePID(10);
  driveInches(5.4, 30);
  wait(1000, msec);
  drivePID(-40);
  inert(-73 * i);
  drivePID(-130);
  

}
void halfAWP(int i) // this is daniel hand redleft and blueright
{
  drivePID(-27);
  driveInchesClamp(-7, 30); // VALUE NEEDS TO BE TWEAKED
  wait(500, msec);
  mIntake.spin(forward, 100, pct);
  /*
  inert(-108*i);
  wait(500, msec);

  drivePID(36);
  wait(500, msec);
  drivePID(-27);
  wait(500,msec);
  */
  inert(-55 * i);
  drivePID(27); //+3; nvm
  wait(500, msec);
  drivePID(-4);

  inert(-145 * i);
  driveInches(12.9,30);
  wait(500, msec);

  drivePID(-25);
  inert(-123 * i);
  drivePID(10);
  driveInches(5.4, 30);

  wait(1000, msec);
  drivePID(-6);
  inert(-145 * i);

  drivePID(-8);
  inert(-55 * i);

  drivePID(-11); //-3; nvm
  drivePID(-22);
  setArmTop();
  inert(-145 * i);
  drivePID(7);
}
void adaptive(int i)
{
  drivePID(-50);
}
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

#include "auton.h"
#include "devices.h"
#include "movement.h"
#include "vex.h"
#include <iostream>
using namespace vex;

void skillsAuto()
{
  drivePIDClamp(-200, 40);
  mIntake.spin(fwd, 100, pct);
  wait(700,msec);
  inert(-90);
  driveInches(35,50);
  inert(135);
  sClamp.set(true);
  drivePID(-27);
  drivePID(25);
  inert(90);
  drivePID(130);
  inert(45);
  drivePID(30);
  drivePID(-30);
  inert(180);
  drivePID(108);
  inert(135);
  drivePID(30);
  drivePID(-30);
  inert(270);
  drivePID(108);
  inert(225);
  drivePID(30);
  drivePID(-40);
  drivePID(30);
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
}
void blueMidAuto(int i)
{
  // Starting position: In blue right corner, facing mobile goal
  drivePID(-27); 
  drivePIDClamp(-300, 30);
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
  drivePIDClamp(-1400,80);
  wait(300,msec);
  inert(125*i);
  mIntake.spin(fwd, 100,pct);
  drivePID(30);
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
  inert(140-34);

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
void AWP()
{ // starts out the same as rightAuto but then goes full field
  driveInchesClamp(-28,50);
  mIntake.spin(fwd,100,pct);
  wait(400, msec);
  inert(-90);
  drivePID(-5);
  sClamp.set(true);
  drivePID(35);
  wait(300,msec);
  mIntake.stop();
  inert(0);
  driveInchesClamp(-17,30);
  drivePID(60);
  mIntake.spin(fwd,100,pct);
  wait(300, msec);
  mIntake.stop();
  inert(135);
  drivePID(-10);
  sClamp.set(true);
  drivePID(10);
  inert(90);
  drivePID(70);
  inert(315);
  driveInchesClamp(34,50);





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
void halfAWP(int i)
{
  drivePID(-27); 
  drivePIDClamp(-300, 30);
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
  setArmTop();   
  inert(-145*i);
  drivePID(7);
}
void adaptive()
{
  drivePID(-50);
  
}
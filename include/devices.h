#ifndef DEVICES_H
#define DEVICES_H

#include "vex.h"

using namespace vex;

// Required device declarations
extern competition Competition;
extern brain Brain;
extern controller Controller1;
extern controller Controller2;

// Motor and sensor device declarations
extern motor mBackRight;
extern motor mBackLeft;
extern motor mFrontRight;
extern motor mFrontLeft;
extern motor mMidLeft;
extern motor mMidRight;
extern motor mIntake;
extern motor mLift;
extern motor mLift2;
extern distance ringCheck;
extern inertial Inertial;
extern rotation Rotation;

// Digital-out device declarations
extern digital_out sClamp;
extern digital_out sDoor;
extern digital_out sintake;

// Limit switch declarations
extern limit limitS;

#endif // DEVICES_H
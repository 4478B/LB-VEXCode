#ifndef DEVICES_H
#define DEVICES_H

#include "vex.h"

using namespace vex;

// Required device declarations
competition Competition;
brain Brain;
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);

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
inertial Inertial = inertial(PORT20);
rotation Rotation = rotation(PORT2);

// Digital-out device declarations
digital_out sClamp = digital_out(Brain.ThreeWirePort.B);
digital_out sDoinker = digital_out(Brain.ThreeWirePort.C);
digital_out sDoor = digital_out(Brain.ThreeWirePort.F);
digital_out sintake = digital_out(Brain.ThreeWirePort.D);


// Limit switch declarations
limit limitS = limit(Brain.ThreeWirePort.E);

#endif // DEVICES_H
#ifndef MAIN_H
#define MAIN_H

#include "vex.h"
#include <iostream>
#include "devices.h"

// Global variables
extern int auton;

// Function to print centered text on the Brain's screen
void printCenter(std::string input);

// Overloaded functions to update the controller screen with PID values
void updateController(double kP, double kI, double kD, int sel, double mag);
void updateController(double mag);
void updateController(double val, double sel, double mag);

// Thread for arm control
vex::thread arm();
#endif // MAIN_H
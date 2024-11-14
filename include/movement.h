#ifndef MOVEMENT_H
#define MOVEMENT_H

// General Utilities
double turnSlew(double val); 
void inertClamp(double t);

// PID Driving
void drivePID(double inches, double kP = 110, double kI = 0, double kD = 0.15, double goalThreshold = 30);
void tunerDrivePID(double inches, double kP = 110, double kI = 0, double kD = 0.15, int ID = -1);
void tunePID();
void GraphPID(double rangeP, double rangeD, double guessP, double guessD, int sqrtTests);

// PID Turning
void inert(double target, double kP = 0.499, double kI = 0, double kD = 0.002);

// Open-loop Driving
void driveDeg(int DDegL, int DDegR, int veloc);
void driveInches(double fwdVal, int veloc);
void drivePIDClamp(double degs, double veloc);
void driveInchesClamp(double fwdVal, int veloc);
void oldDrivePID(double degs, double veloc);

// Odometry
void odometry();

// Arm Control
void setArm(int armPos);
void setArmBottom();
void setArmMid();
void setArmTop();

#endif // MOVEMENT_H
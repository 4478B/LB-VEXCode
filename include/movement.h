#ifndef MOVEMENT_H
#define MOVEMENT_H

// Helper functions
double turnSlewStep;
double turnSlew(double val);
void inert(double target, double kP = 0.475, double kI = 0, double kD = 0.002);
void driveDeg(int DDegL, int DDegR, int veloc);
void drivePID(double inches, double kP = 110, double kI = 0, double kD = .15, double wheelRadius = 1.375);
void tunerDrivePID(double inches, double kP = 110, double kI = 0, double kD = .15, int ID = -1);
void oldDrivePID(double degrs, double veloc);
void drivePIDClamp(double degrs, double veloc);
void driveInches(double fwdVal, int veloc);
void driveInchesClamp(double fwdVal, int veloc);
void tunePID(void);
void GraphPID(double rangeP, double rangeD, double guessP, double guessD, int sqrtTests);
void setArmBottom();
void setArmMid();
void setArmTop();

#endif // MOVEMENT_H
#include "odometry.h"
#include "devices.h"
#include "vex.h"

#include <iostream>
#include <iomanip>
using namespace vex;

// constants used in functions
const double WHEEL_RADIUS = 1.375;
const int WHEEL_OUTPUT = 48;
const int GEAR_INPUT = 36;

// distance between right wheel and tracking center [inches?]
const double rDist = 5;

// initial angle robot starts at (specified in odomSelector) [radians]
double initTheta = 0;

// positions of parallel wheels used in odometry loop [inches]
double currentR = 0;
double prevR = 0;

// displacement of wheel between loops [inches]
double deltaR = 0;

// angles of bot used in odometry loop [radians]
double theta;
double prevTheta;

// change in angle of bot between loops [radians]
double deltaTheta = 0;


// average theta throughout arc in each loop [radians]
double avgTheta;

// local displacement in each loop [inches]
double deltaLocal = 0;

// local displacement mapped to (x,y) [inches]
double deltaXGlobal = 0;
double deltaYGlobal = 0;

// global position of the robot [inches]
double posX;
double posY;

// variable representing if odometry is running
bool odomRunning = false;
// variable representing if odometry is being calculated
bool odomLoopActive = false;

void odometry() {
  setStartingOdomValues();
  odomRunning = true;

  while(odomRunning) {  
    odomLoopActive = true;

    // Gets the angle of the right wheel [radians]
    currentR = mMidRight.position(deg) * M_PI / 180;
    // calculates change in wheel position since last loop [inches]
    deltaR = (currentR - prevR) * WHEEL_RADIUS;
    // Updates positions for next loop
    prevR = currentR;

    // Gets the orientation of robot [radians]
    theta = Inertial.heading() * M_PI / 180;
    // Wraps theta around domain of [0,2pi)
    if(theta < 0 || theta > 2 * M_PI){
      theta = fmod(theta,2*M_PI);
    }

    // Calculates change in orientation since last loop [radians]
    deltaTheta = theta - prevTheta;
    // Updates positions for next loop
    prevTheta = theta;

    if(deltaTheta == 0) {
      // If robot didn't turn, movement is straightforward
      deltaLocal = deltaR;
    } else {
      // Otherwise, use law of sines to solve for movement
      deltaLocal = 2 * ((deltaR / deltaTheta) - rDist) * sin(deltaTheta / 2.0);
    }

    // Calculates average angle of theta in arc (loop) [radians]
    avgTheta = theta - (deltaTheta / 2);

    // Splits local displacement into X and Y factors
    deltaXGlobal = (deltaLocal * cos(avgTheta));
    deltaYGlobal = (deltaLocal * sin(avgTheta));

    //* Updates the global position
    posX += deltaXGlobal;
    posY += deltaYGlobal;

    odomLoopActive = false;
    wait(10,msec);
  }
}

// sets starting values based on auton configs
void setStartingOdomValues() {
  /*switch (autonSelection) {
      case 0:
        // WIP
        posX = 0;
        posY = 0;
        initTheta = 0;
        break;
  }*/
  posX = 0;
  posY = 0;
  initTheta = 0;
  theta = initTheta;
  prevTheta = theta;
  avgTheta = theta + (deltaTheta / 2);
}

void odomKillSwitch() {
  odomRunning = !odomRunning;
}

void odomDataCollection() {
    // Set up formatting for the entire output stream
    std::cout << std::fixed << std::setprecision(2);
    
    // Print header using setw for consistent column widths
    std::cout << std::setw(14) << "theta" << " | "
              << std::setw(14) << "deltaTheta" << " | "
              << std::setw(14) << "avgTheta" << " | "
              << std::setw(14) << "posX" << " | "
              << std::setw(14) << "posY" << std::endl;
              
    // Print separator line
    std::cout << std::string(15 * 5 + 4, '-') << std::endl;
    
    // Print data values in a loop
    while (odomRunning) {
        // Convert theta to degrees and print all values
        std::cout << std::setw(14) << (theta * 180 / M_PI) << " | "
                  << std::setw(14) << (deltaTheta * 180 / M_PI) << " | "
                  << std::setw(14) << (avgTheta * 180 / M_PI) << " | "
                  << std::setw(14) << posX << " | "
                  << std::setw(14) << posY << std::endl;
                  
        wait(500, msec);
    }
}
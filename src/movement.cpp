#include "movement.h"
#include "devices.h"
#include "vex.h"
#include "main.h"
using namespace vex;

// constants used in functions
const double WHEEL_RADIUS = 1.375;
const int WHEEL_OUTPUT = 48;
const int GEAR_INPUT = 36;

double turnSlewStep = 6;

double turnSlew(double val)
{
  static double prevVal = 0;
  if (prevVal + turnSlewStep < val)
  {
    prevVal = prevVal + turnSlewStep;
    return prevVal;
  }
  prevVal = val;
  return val;
}

void inert(double target, double kP, double kI, double kD)
{
  bool isComplete = false;
  double startTime = Brain.timer(msec), prevTime = startTime, deltaTime, currentTime;
  double integral = 0, error = 0, derivative = 0, prevError = 0;
  double currentDeg = Inertial.heading(degrees);
  double output;
  int oscillation = 0;
  while (!isComplete)
  {
    //////////////////setting values
    currentTime = Brain.timer(sec);
    deltaTime = currentTime - prevTime;
    prevTime = currentTime;
    currentDeg = Inertial.heading(degrees);

    // calculations

    error = target - currentDeg;

    if (error > 180)
    {
      error = error - 360;
    }
    else if (error < -180)
    {
      error = 360 + error;
    }

    derivative = (error - prevError) / deltaTime;

    prevError = error;

    integral = +error * deltaTime;

    output = kP * error + kI * integral + kD * derivative;

    //  output = turnSlew(output);

    //////////////////////// motor output

    leftMotors.spin(forward, output, percent);
    rightMotors.spin(forward, -output, percent);

    //      Printing values

    //      Exiting loop
    if (((fabs(prevError) < 1.5)))
    {
      oscillation++;
      if (oscillation > 1)
      {
        isComplete = true;
        allMotors.stop(hold);
        break;
      }
    }
    vex ::wait(20, msec);
  }
}

void inertClamp(double t)
{
  inert(t, 0.499, 0.0, 0.002);
}

void drivePID(double inches, double kP, double kI, double kD, double goalThreshold)
{
  // Function to control robot movement using PID
  int inGoal = 0;                                      // Tracks robot's time in goal threshold
  double currentDelta;                                 // Error between target and current position
  double P = 0, I = 0, D = 0, totalPID;                // PID terms
  double pollingRate = 20;                             // Polling rate in ms
  double target = inches * (360 / (2 * M_PI * 1.375)); // Target position in degrees (1.375 is wheelRadius)

  inches *= WHEEL_OUTPUT / GEAR_INPUT; // Account for gear ratio

  double previousDelta = target; // Initialize previous error as target
  double integralSum = 0;        // Cumulative error for integral term

  double startTime = Brain.Timer.time();
  double timeout = 3000;                                 // max time before PID times out
  double goalsNeeded = (fabs(inches) / 5) * pollingRate; // makes time spent in goal proportional to distance
  if (goalsNeeded == 0)
  { // sets bounds (max & min) for goals needed to reach goal
    goalsNeeded = 1;
  }
  else if (goalsNeeded > 5)
  {
    goalsNeeded = 5;
  }

  // Reset motor encoder value to 0
  allMotors.setPosition(0, degrees);

  while (inGoal < goalsNeeded) // CHECK IF IT SHOULD BE A < or <=
  {
    // Main PID loop; runs until target is reached
    // Read motor position (you can average left and right motor values for straight driving)
    double currentPosition = (mBackLeft.position(degrees) + mBackRight.position(degrees) + mFrontLeft.position(degrees) + mMidLeft.position(degrees) + mMidRight.position(degrees) + mFrontRight.position(degrees)) / 6;

    // Calculate the current error
    currentDelta = target - currentPosition;

    // Proportional: Larger error results in larger response
    P = (kP / 1000) * currentDelta;

    // Integral: Sum of all errors helps correct for small errors over time
    integralSum += currentDelta;
    I = kI * integralSum;

    // Derivative: React to the rate of error change
    D = kD * (currentDelta - previousDelta) / pollingRate;

    // Calculate total PID response
    totalPID = P + I + D;

    // Use totalPID to move motors proportionally
    allMotors.spin(forward, totalPID, percent);

    // Check if the error is small enough to stop
    if (fabs(currentDelta) < goalThreshold)
    {
      inGoal++;
    }
    else
    {
      inGoal = 0;
    }
    // Check if should timeout
    if ((Brain.Timer.time() - startTime) >= timeout)
    {
      break;
    }

    // Update the previous error for the next loop
    previousDelta = currentDelta;

    // Wait for the polling rate before next iteration
    wait(pollingRate, msec);
  }
  // Stop the motors once goal is met
  allMotors.stop();
}

void tunerDrivePID(double inches, double kP, double kI, double kD, int ID)
{
  // Function to control robot movement using PID
  int goalMet = 0;                                     // Flag to track if the goal is met
  double currentDelta;                                 // Error between target and current position
  double P = 0, I = 0, D = 0, totalPID;                // PID terms
  double pollingRate = 20;                             // Polling rate in ms
  double target = inches * (360 / (2 * M_PI * 1.375)); // Target position in degrees
  double startTime = Brain.Timer.time();               // TUNER EXCLUSIVE: starting time for timeout
  double timeElapsed = 0;                              // TUNER EXCLUSIVE: time elapsed since starting PID
  double currentPitch = Inertial.pitch();
  double maxPitch = 0;

  double previousDelta = target; // Initialize previous error as target
  double integralSum = 0;        // Cumulative error for integral term

  std::cout << "target = " << target << std::endl; // TUNER EXCLUSIVE: prints goal to console for graphing

  // Reset motor encoder value to 0
  mBackLeft.setPosition(0, degrees);
  mFrontLeft.setPosition(0, degrees);
  mMidLeft.setPosition(0, degrees);
  mFrontRight.setPosition(0, degrees);
  mBackRight.setPosition(0, degrees);
  mMidRight.setPosition(0, degrees);

  while (goalMet <= 1 && timeElapsed <= 5000)
  { // Main PID loop; runs until target is reached
    // Read motor position (you can average left and right motor values for straight driving)
    double currentPosition = (mBackLeft.position(degrees) + mBackRight.position(degrees) + mFrontLeft.position(degrees) + mMidLeft.position(degrees) + mMidRight.position(degrees) + mFrontRight.position(degrees)) / 6;

    // Calculate the current error
    currentDelta = target - currentPosition;

    // Proportional: Larger error results in larger response
    P = (kP / 1000) * currentDelta;

    // Integral: Sum of all errors helps correct for small errors over time
    integralSum += currentDelta;
    I = kI * integralSum;

    // Derivative: React to the rate of error change
    D = kD * (currentDelta - previousDelta) / pollingRate;

    // Calculate total PID response
    totalPID = P + I + D;

    // Use totalPID to move motors proportionally
    allMotors.spin(forward, totalPID, percent);

    // Check if the error is small enough to stop
    if (fabs(currentDelta) < 30)
    {
      goalMet++;
    }
    // Update the previous error for the next loop
    previousDelta = currentDelta;

    // TUNER EXCLUSIVE: time elapsed for timeout
    timeElapsed = Brain.Timer.time() - startTime;

    // TUNER EXCLUSIVE: Print coordinates to console for graphing
    if (ID == -1)
    {
      std::cout << "(" << timeElapsed
                << "," << currentPosition << "," << currentPitch << ")" << std::endl;
    }
    else
    { // GRAPH EXCLUSIVE: finds maximum pitch of robot
      if (currentPitch > maxPitch)
      {
        maxPitch = currentPitch;
      }
    }

    // Wait for the polling rate before next iteration
    wait(pollingRate, msec);
  }
  // Stop the motors once goal is met
  allMotors.stop();
  // GRAPH EXCLUSIVE: print coordinates for advanced graphing
  if (ID != -1)
  {
    double maxPitch = 1; // TEMP
    double motorHeat = (mBackLeft.position(degrees) + mBackRight.position(degrees) + mFrontLeft.position(degrees) + mMidLeft.position(degrees) + mMidRight.position(degrees) + mFrontRight.position(degrees)) / 6;
    std::cout << "(" << ID << "," << kP << "," << kD << ","
              << timeElapsed << "," << maxPitch << ","
              << motorHeat << std::endl;
  }
}

void driveInches(double inches, int veloc, bool clamping)
{

  // adjusted inches based on gear ratios
  double adjustedInches = inches * (WHEEL_OUTPUT / GEAR_INPUT);

  // conversion from inches to degrees
  double degrs = (adjustedInches * 180) / (WHEEL_RADIUS * M_PI);

  double average = 0;
  allMotors.setPosition(0, degrees);

  // Determine direction of movement
  directionType moveDirection = (degrs >= 0) ? forward : reverse;
  // Use absolute value for comparisons
  double targetDegrees = fabs(degrs);
  double slowdownThreshold = (degrs >= 0) ? 250 : 150;

  while ((moveDirection == forward && average < degrs) ||
         (moveDirection == reverse && average > degrs))
  {
    // Calculate average position
    average = (mBackLeft.position(degrees) +
               mBackRight.position(degrees) +
               mFrontLeft.position(degrees) +
               mMidLeft.position(degrees) +
               mMidRight.position(degrees) +
               mFrontRight.position(degrees)) /
              6;

    // Spin motors in appropriate direction
    allMotors.spin(moveDirection, veloc, pct);

    // Handle slowdown and clamp
    double distanceRemaining = fabs(degrs - average);
    if (veloc > 15 && distanceRemaining < slowdownThreshold)
    {
      veloc = veloc * (distanceRemaining / targetDegrees);
      // Set clamp opposite to current value
      if (clamping)
      {
        sClamp.set(!sClamp.value());
      }
    }

    vex::wait(10, msec);
  }
}

// alias for clamping mode
void driveInchesClamp(double inches, int veloc) { driveInches(inches, veloc, true); }

void tunePID()
{
  using std::cout;
  using std::endl;
  double tempPID[3] = {110, 0, 0.15};           // Starting values for PID
  const double resetVal = 0.1, resetMag = 0.1;  // Defines what values to reset to
  double valMag = resetMag;                     // Sets initial magnitude
  int currentConst = 0;                         // Defines current constant (P,I, or D) to change
  cout << "Starting P: " << tempPID[0] << endl; // logs starting values to console
  cout << "Starting I: " << tempPID[1] << endl;
  cout << "Starting D: " << tempPID[2] << endl;
  cout << "Now modifying P" << endl; // tells user default value modified
  updateController(tempPID[0], tempPID[1], tempPID[2], 0, valMag);
  while (true)
  {

    if (Controller1.ButtonUp.pressing() && Controller1.ButtonDown.pressing())
    {
      valMag = resetMag; // Resets magnitude of dynamic change
      cout << "magnitude RESET to " << resetMag << endl;
      updateController(resetMag);
    }
    else if (Controller1.ButtonUp.pressing())
    {
      valMag *= 10; // Increments magnitude by a factor of 10
      cout << "magnitude set to " << valMag << endl;
      updateController(valMag);
    }
    else if (Controller1.ButtonDown.pressing())
    {
      valMag /= 10; // Decrements magnitude by a factor of 10
      cout << "magnitude set to " << valMag << endl;
      updateController(valMag);
    }

    if (Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing())
    {
      tempPID[currentConst] = resetVal; // Resets temp. PID value
      cout << "value RESET to " << resetVal << endl;
      updateController(resetVal, currentConst, valMag);
    }
    else if (Controller1.ButtonLeft.pressing())
    {
      tempPID[currentConst] -= valMag; // Decreases temp. PID value by magnitude
      cout << "value set to " << tempPID[currentConst] << endl;
      updateController(tempPID[currentConst], currentConst, valMag);
    }
    else if (Controller1.ButtonRight.pressing())
    {
      tempPID[currentConst] += valMag; // Increases temp. PID value by magnitude
      cout << "value set to " << tempPID[currentConst] << endl;
      updateController(tempPID[currentConst], currentConst, valMag);
    }

    if (Controller1.ButtonY.pressing())
    { // Controls value being changed
      currentConst += 1;
      currentConst %= 3; // variable has 3 states, modulus keeps within 3 states
      if (currentConst == 0)
      {
        cout << "Now modifying P" << endl;
      }
      else if (currentConst == 1)
      {
        cout << "Now modifying I" << endl;
      }
      else
      {
        cout << "Now modifying D" << endl;
      }
      updateController(tempPID[0], tempPID[1], tempPID[2], currentConst, valMag);
    }
    if (Controller1.ButtonA.pressing())
    { // Starts test with the given variables
      cout << "Starting test with following P,I,D: " << tempPID[0] << "," << tempPID[1] << "," << tempPID[2] << endl;
      tunerDrivePID(24 * 2, tempPID[0], tempPID[1], tempPID[2]); // Tests PID by moving 2 tiles WITH LOGGING
      wait(2, sec);
      inert(Inertial.rotation() + 180); // Turns robot so it is repeatable
      wait(1, sec);
      cout << "Test complete" << endl;
    }
    wait(100, msec); // pause between input detection to not double them accidentally
  }
}

void GraphPID(double rangeP, double rangeD, double guessP, double guessD, int sqrtTests)
{
  double currentP = guessP - rangeP; // Values representing current search boundaries
  double currentD = guessD - rangeD;
  double minP = currentP; // Search starts at the minimum, goes to maximum
  double deltaP = rangeP / sqrtTests;
  double deltaD = rangeD / sqrtTests;
  int maxID = sqrtTests ^ 2 - 1;
  int ID = 0; // IDs represent individual tests to make them replicable
  std::cout << "What is current ID (0 if starting): ";
  std::cin >> ID; // retrieves user input so it can be run in mutliple sessions
  std::cout << std::endl;
  while (ID <= maxID)
  {

    tunerDrivePID(100, currentP, 0, currentD, ID); // Custom PID with more logging

    if (ID % 10 == 9) // This is the loop that creates the gridlike pattern
    {
      currentD += deltaD;
      currentP = minP;
    }
    else
    {
      currentP += deltaP;
    }
    ID++; // assigns new ID so next test can start
  }
}

void setArm(int targetDeg)
{

  // PID Constants
  const double kP = 1500;
  const double kI = 0;
  const double kD = 0.01;

  // PID Variables
  double P = 0, I = 0, D = 0, totalPID;
  double currentDelta, previousDelta = 0;
  double integralSum = 0;
  int goalMet = 0;

  // Position tracking variables
  double currentPosition;
  double error;

  // double startTime = Brain.Timer.time();
  // double timeout = 200;
  // double timeElapsed;

  // Reset motor encoder
  mLift.setPosition(0, degrees);

  while (goalMet <= 1)
  {
    currentPosition = Rotation.angle(deg);
    error = targetDeg - currentPosition;

    // Normalize error to [-180, 180] range
    if (error > 180)
    {
      error -= 360;
    }
    else if (error < -180)
    {
      error += 360;
    }

    currentDelta = error;

    // PID calculations
    P = (kP / 1000) * currentDelta;
    integralSum += currentDelta;
    I = kI * integralSum;
    D = (kD / 20) * (currentDelta - previousDelta);

    totalPID = P + I + D;

    // Apply motor movement
    armMotors.spin(forward, totalPID, percent);

    if (fabs(currentDelta) < 2)
    {
      goalMet++;
    }

    previousDelta = currentDelta;

    // timeElapsed = Brain.Timer.time() - startTime;

    wait(20, msec);
  }

  // Stop motors
  armMotors.stop(hold);
}

void colorSortRed() // 1 is for red, 2 is for blue
{
  mIntake.setVelocity(100, pct);
  colorSens.setLightPower(100, percent);
  colorSens.setLight(ledState::on);
  int armPos = 1;
  while (true)
  {
    if (armPos == 2)
    {
      setArm(0);
      armPos = 1;
    }
    mIntake.spin(fwd, 100, pct);
    if ((colorSens.hue() > 330 || colorSens.hue() < 45) && colorSens.isNearObject())
    {
      // mIntake.spinFor(fwd, 1000, deg);
      wait(10, msec);
      // setArm(30);
      armPos = 2;
      mIntake.stop(hold);
      mIntake.spin(reverse, 100, pct);
      wait(50, msec);
    }
    wait(10, msec);
  }
}
// aliases for specific positions
void setArmBottom() { setArm(0); }
void setArmMid() { setArm(37); }
void setArmTop() { setArm(134); }

const double SMOOTHING_DENOMINATOR = 100; // Used to normalize the exponential curve
const double EXPONENTIAL_POWER = 2;         // Controls how aggressive the curve is
// Helper function that makes joystick input more precise for small movements
// while maintaining full power at maximum joystick
double logDriveJoystick(double joystickPCT)
{
  // Get the absolute value for calculation
  double magnitude = fabs(joystickPCT);

  // Calculate the smoothed value
  double smoothedValue = pow(magnitude, EXPONENTIAL_POWER) / SMOOTHING_DENOMINATOR;

  // Restore the original sign (positive or negative)
  return joystickPCT >= 0 ? smoothedValue : -smoothedValue;
}

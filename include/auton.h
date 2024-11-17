#ifndef AUTON_H
#define AUTON_H

#include <functional>

// Structure to hold autonomous routine information
struct AutonRoutine {
    const char* displayName;
    std::function<void(int)> routine;
    int multiplier;
};

class AutonSelector {
private:
    static const AutonRoutine routines[];  // Array declaration
    int currentSelection;  // Default to AWP (Route 5)
    const int routineCount;

public:
    void nextSelection();
    void displayCurrentSelection();
    void runSelectedAuton();
    int getCurrentSelection() const;
};

// Declare the global instance that will be used by other files
extern AutonSelector autonSelector;


// Forward declarations for autonomous routes
void skillsAuto(int i);
void blueRightAuto(int i);
void blueMidAuto(int i);
void redleftAuto(int i);
void rightAuto(int i);
void AWP(int i);
void halfAWP(int i);
void adaptive();

#endif // AUTON_H
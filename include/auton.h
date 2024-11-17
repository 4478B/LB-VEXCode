#ifndef AUTON_H
#define AUTON_H

#include <functional>

// Forward declarations of autonomous functions
void rightAuto(int direction);
void halfAWP(int direction);
void blueRightAuto(int direction);
void skillsAuto(int direction);
void AWP(int direction);
void blueMidAuto(int direction);

// Forward declaration of internal structs/classes
struct AutonRoutine;

class AutonSelector {
private:
    const AutonRoutine* routines;  // Pointer to array of routines
    int currentSelection;          // Current selected routine
    int routineCount;             // Total number of routines

public:
    AutonSelector();  // Constructor
    void nextSelection();
    void displayCurrentSelection();
    void runSelectedAuton();
    int getCurrentSelection() const;
};

// Global instance getter
AutonSelector& getAutonSelector();


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
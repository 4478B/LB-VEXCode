# Masuk High School VEX Team 4478B
## High Stakes 2024-2025 Season

<img src="/api/placeholder/800/400" alt="Robot Overview" />

## Robot Overview
Our robot for the 2024-2025 VEX High Stakes season features an innovative through-bot intake system optimized for efficient scoring. Key features include:

### Mechanical Specifications
- Integrated through-bot intake system with direct mobile goal scoring capability
- Precision arm mechanism for wall stake scoring
- Advanced clamp mechanism for mobile goal control
- Strategic 6-motor configuration with specialized half-motor implementation for arm stability

### Hardware Configuration
```cpp
// Drive Motors
PORT5: Front Right (6:1)
PORT6: Mid Right (6:1)
PORT7: Back Right (6:1)
PORT8: Front Left (6:1)
PORT9: Mid Left (6:1)
PORT10: Back Left (6:1)

// Mechanism Motors
PORT1: Intake (6:1)
PORT3: Lift 1 (18:1)
PORT4: Lift 2 (18:1)

// Sensors
PORT17: Ring Detection Distance Sensor
PORT20: Inertial Sensor
PORT2: Rotation Sensor

// Digital Ports
Port B: Clamp
Port F: Door
Port D: Intake
Port E: Limit Switch
```

### Motor Groups
- **Left Drive**: Front, Mid, and Back Left motors
- **Right Drive**: Front, Mid, and Back Right motors
- **All Drive**: Combined left and right motor groups
- **Arm System**: Dual lift motors (18:1 ratio for precision)

## Technical Implementation

### Movement Systems
The robot employs sophisticated movement control systems:

#### PID Control Systems
- **Multi-Resolution PID Tuning**
  - Optimizes P and D values using minimum pitch change criteria
  - Minimizes time-to-target optimization
  - Live controller-based tuning interface
  - Automatic recentering for repeated testing cycles

#### Movement Functions
```cpp
// PID Control
void drivePID(double inches, double kP = 110, double kI = 0, double kD = 0.15, double goalThreshold = 30);
void inert(double target, double kP = 0.499, double kI = 0, double kD = 0.002);

// Arm Control
void setArm(int armPos);
void setArmBottom();
void setArmMid();
void setArmTop();

// Advanced Movement
void odometry();
void setStartingOdomValues();
double logDriveJoystick(double joystickPCT);
```

### Project Structure
```
src/
├── auton.cpp        # Autonomous routines
├── devices.cpp      # Device configurations
├── main.cpp         # Primary control flow
├── movement.cpp     # Movement algorithms
└── headers/
    ├── movement.h   # Movement system declarations
    └── devices.h    # Hardware configuration
```

### Key Features
- **PID Tuning System**
  - Multi-resolution search algorithm
  - Real-time parameter adjustment
  - Automatic testing cycle support
- **Odometry Implementation**
  - Position tracking
  - Automated kill switch
  - Data collection routines
- **Movement Control**
  - Turn slew rate control
  - Inertial clamping
  - Logarithmic joystick control

## Competition Strategy
Our team focuses on:
- Maintaining control of goals in the positive corner
- Late-game wall stake scoring
- Strategic mobile goal manipulation

### Achievements
- Excellence Award - SAINTS V5RC Competition

## Development Guidelines

### Code Management
- Private repository with controlled access
- Feature-based branch management
- Regular testing and validation cycles

### Best Practices
- Maintain established file structure
- Comprehensive commenting for new implementations
- Modular feature development in separate files
- Extensive route testing pre-competition
- Regular PID optimization using 3D tuner or manual configuration

## Setup and Installation
1. Clone repository
2. Verify device configuration matches team setup:
   - Confirm motor ports and configurations
   - Verify sensor connections
   - Check digital port assignments
3. Deploy to robot

## Resources and Documentation
- [Engineering Notebook](https://docs.google.com/document/d/1_MTpEjIHr-jA4OIaVsaplkgD773AaJuhlHmIqOZQlZI/edit?tab=t.0) (Access required)
- [Design Documentation] (Placeholder)
- [CAD Files] (Placeholder)

## Team Information
### Maintainers
- Evan Boyle
- Aryan Sharma

### Contact Information
[Placeholder for contact details]

### Season Timeline
[Placeholder for development milestones]

---

*Last Updated: November 2024*

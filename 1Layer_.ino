#include <AccelStepper.h>

// --------------------------------------------------------
//   Pin/Driver Setup
// --------------------------------------------------------
#define motorInterfaceType AccelStepper::DRIVER

// Spool motor pins
#define SPOOL_STEP_PIN  3
#define SPOOL_DIR_PIN   2

// Wire guide motor pins
#define GUIDE_STEP_PIN  4
#define GUIDE_DIR_PIN   5

// Create stepper objects
AccelStepper spoolMotor(motorInterfaceType, SPOOL_STEP_PIN, SPOOL_DIR_PIN);
AccelStepper guideMotor(motorInterfaceType, GUIDE_STEP_PIN, GUIDE_DIR_PIN);

// --------------------------------------------------------
//   Winding Parameters for a Single Layer
// --------------------------------------------------------

// How many steps does the spool motor take for one layer
const long SPOOL_STEPS = 6000; 

// How many steps does the wire guide move (30 mm) for one layer
const long GUIDE_STEPS = 14300;

// --------------------------------------------------------
//   Speeds / Accelerations
// --------------------------------------------------------
// Adjust these values as needed for your hardware.
const float SPOOL_MAX_SPEED = 800.0; // spool: steps/second
const float SPOOL_ACCEL     = 200.0; // spool: steps/second^2

// Using the same ratio as before (14300 / 6000 ~ 2.38)
const float GUIDE_MAX_SPEED = 2.38 * SPOOL_MAX_SPEED; // ~715 steps/second
const float GUIDE_ACCEL     = 2.38 * SPOOL_ACCEL;     // ~715 steps/second^2

// --------------------------------------------------------
//   State Variable
// --------------------------------------------------------
bool isRunning = false; // Are we currently winding?

// --------------------------------------------------------
//   Setup
// --------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println("Type 'run' to begin single layer winding, 'stop' to halt early.");

  // Set up spool motor speeds & acceleration
  spoolMotor.setMaxSpeed(SPOOL_MAX_SPEED);
  spoolMotor.setAcceleration(SPOOL_ACCEL);

  // Set up guide motor speeds & acceleration
  guideMotor.setMaxSpeed(GUIDE_MAX_SPEED);
  guideMotor.setAcceleration(GUIDE_ACCEL);

  // If your motors run in the wrong directions, try inverting pins:
  // spoolMotor.setPinsInverted(false, false, false);
  // guideMotor.setPinsInverted(false, false, false);
}

// --------------------------------------------------------
//   Start the Winding Process for a Single Layer
// --------------------------------------------------------
void startWinding() {
  isRunning = true;
  
  // Reset positions to 0 (assuming a 'home' or start position)
  spoolMotor.setCurrentPosition(0);
  guideMotor.setCurrentPosition(0);

  // Command the motors to move:
  // - Spool motor: move forward by SPOOL_STEPS.
  // - Wire guide: move forward by GUIDE_STEPS.
  spoolMotor.moveTo(SPOOL_STEPS);
  guideMotor.moveTo(GUIDE_STEPS);

  Serial.print("Starting winding: spoolMotor moving to ");
  Serial.print(SPOOL_STEPS);
  Serial.print(" steps, guideMotor moving to ");
  Serial.print(GUIDE_STEPS);
  Serial.println(" steps.");
}

// --------------------------------------------------------
//   Main Loop
// --------------------------------------------------------
void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("run")) {
      Serial.println("Starting single layer winding...");
      startWinding();
    }
    else if (cmd.equalsIgnoreCase("stop")) {
      Serial.println("Stopping early...");
      // Stop the motors (this decelerates them to zero)
      spoolMotor.stop();
      guideMotor.stop();
      isRunning = false;
    }
  }

  // If winding is in progress, run the motors
  if (isRunning) {
    spoolMotor.run();
    guideMotor.run();

    // Check if both motors have finished their moves
    if (spoolMotor.distanceToGo() == 0 && guideMotor.distanceToGo() == 0) {
      Serial.println("Single layer complete!");
      isRunning = false;  // Stop further processing
      
      // Return the wire guide motor to its home position (0)
      guideMotor.moveTo(0);
      // Continue running the guide motor until it reaches home
      while (guideMotor.distanceToGo() != 0) {
        guideMotor.run();
      }
      Serial.println("Wire guide motor returned to home position.");
    }
  }
}

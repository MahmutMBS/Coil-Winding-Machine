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
//   Winding Parameters
// --------------------------------------------------------
const long FIRST_LAYER_SPOOL_STEPS      = 6000; // spool steps for the first layer
const long SUBSEQUENT_LAYER_SPOOL_STEPS = 6000; // spool steps for subsequent layers

// Baseline guide target for a 6000-step layer
const long BASE_GUIDE_TARGET = 14300;

// Compute effective ratios for guide motor (hardcoded for simplicity)
//const float effectiveGuideRatio1 = (float)BASE_GUIDE_TARGET / FIRST_LAYER_SPOOL_STEPS;
const float effectiveGuideRatio1 = 2.5;
//const float effectiveGuideRatio2 = (float)BASE_GUIDE_TARGET / SUBSEQUENT_LAYER_SPOOL_STEPS;
const float effectiveGuideRatio2 = 2.5;

// --------------------------------------------------------
//   Speeds & Accelerations
// --------------------------------------------------------
// FIRST layer spool speeds/accels
const float SPOOL_SPEED_LAYER1  = 750.0;  // steps/sec
const float SPOOL_ACCEL_LAYER1  = 500.0;  // steps/sec^2

// SUBSEQUENT layers spool speeds/accels
const float SPOOL_SPEED_OTHERS  = 750.0;  // steps/sec
const float SPOOL_ACCEL_OTHERS  = 300.0;  // steps/sec^2

// We'll calculate the guide motor speeds/accels as a ratio of the spool’s
// so that guide stays in sync for each layer:
float guideSpeedLayer1    = 0.0;
float guideAccelLayer1    = 0.0;
float guideSpeedOthers    = 0.0;
float guideAccelOthers    = 0.0;

// --------------------------------------------------------
//   Unified Final Deceleration
// --------------------------------------------------------
const float FINAL_ACCEL = 2000.0;  // Used by both spool & guide for fast decel

// Separate thresholds if desired
const long GUIDE_FINAL_THRESHOLD = 500; // steps
const long SPOOL_FINAL_THRESHOLD = 60;  // steps

// --------------------------------------------------------
//   Other Winding Settings
// --------------------------------------------------------
const long GUIDE_RETRACTION_STEPS = 0; // example retraction
const int  TOTAL_LAYERS           = 7;

// --------------------------------------------------------
//   State Variables
// --------------------------------------------------------
bool isRunning            = false;
int  currentLayer         = 0;
bool finalDecelerationSet = false;
bool isRetractingGuide    = false;

// Pause state variables
bool paused               = false;
long spoolRemaining       = 0;
long guideRemaining       = 0;

// NEW: Adjustment mode state (applies only for layer 2)
bool waitingForAdjustment = false;

// Variables for the current layer
long  currentSpoolSteps          = 0;
long  currentGuideTarget         = 0;
int   currentGuideDirection      = 1;  // 1 forward, -1 reverse
long  startGuidePosition         = 0;  // guide start position
float currentEffectiveGuideRatio = 0.0;

// --------------------------------------------------------
//   Function to Continue to Layer 2 After Adjustment
// --------------------------------------------------------
void continueToLayer2() {
  // Set parameters for subsequent layers (layer 2 in this case)
  spoolMotor.setMaxSpeed(SPOOL_SPEED_OTHERS);
  spoolMotor.setAcceleration(SPOOL_ACCEL_OTHERS);
  
  guideSpeedOthers = effectiveGuideRatio2 * SPOOL_SPEED_OTHERS;
  guideAccelOthers = effectiveGuideRatio2 * SPOOL_ACCEL_OTHERS;
  guideMotor.setMaxSpeed(guideSpeedOthers);
  guideMotor.setAcceleration(guideAccelOthers);

  // Reverse guide direction if desired (as in the original logic)
  startGuidePosition = guideMotor.currentPosition();
  currentGuideDirection *= -1;
  currentSpoolSteps = SUBSEQUENT_LAYER_SPOOL_STEPS;
  currentEffectiveGuideRatio = effectiveGuideRatio2;

  // Re-zero spool motor for the new layer
  spoolMotor.setCurrentPosition(0);

  // Calculate new targets for spool and guide motors
  currentGuideTarget = (long)(currentSpoolSteps * currentEffectiveGuideRatio);
  long targetSpool = currentSpoolSteps;
  long targetGuide = startGuidePosition + currentGuideDirection * currentGuideTarget;

  spoolMotor.moveTo(targetSpool);
  guideMotor.moveTo(targetGuide);

  finalDecelerationSet = false;
}

// --------------------------------------------------------
//   Function to Start a New Layer
// --------------------------------------------------------
void startNewLayer() {
  // Check if we've finished all layers
  if (currentLayer >= TOTAL_LAYERS) {
    Serial.println("All layers complete. Returning to home position.");
    spoolMotor.moveTo(0);
    guideMotor.moveTo(0);
    isRunning = false;
    return;
  }

  currentLayer++;
  Serial.print("Starting layer ");
  Serial.println(currentLayer);

  if (currentLayer == 1) {
    // FIRST Layer
    spoolMotor.setMaxSpeed(SPOOL_SPEED_LAYER1);
    spoolMotor.setAcceleration(SPOOL_ACCEL_LAYER1);

    // Calculate guide speeds based on ratio * spool
    guideSpeedLayer1 = effectiveGuideRatio1 * SPOOL_SPEED_LAYER1;
    guideAccelLayer1 = effectiveGuideRatio1 * SPOOL_ACCEL_LAYER1;
    guideMotor.setMaxSpeed(guideSpeedLayer1);
    guideMotor.setAcceleration(guideAccelLayer1);

    // For the first layer, reset the guide position to 0
    guideMotor.setCurrentPosition(0);
    startGuidePosition         = 0;
    currentGuideDirection      = 1;
    currentSpoolSteps          = FIRST_LAYER_SPOOL_STEPS;
    currentEffectiveGuideRatio = effectiveGuideRatio1;
  } else {
    // For subsequent layers:
    if (currentLayer == 2) {
      // For the second layer, enter adjustment mode.
      waitingForAdjustment = true;
      Serial.println("Layer 1 complete.");
      Serial.println("Enter 'adjust <offset>' (e.g., 'adjust 100' or 'adjust -50') to move the guide,");
      Serial.println("then type 'continue' to start layer 2.");
      return; // Wait until adjustment is done.
    } else {
      // For layers 3 and beyond, proceed as before.
      spoolMotor.setMaxSpeed(SPOOL_SPEED_OTHERS);
      spoolMotor.setAcceleration(SPOOL_ACCEL_OTHERS);
      
      guideSpeedOthers = effectiveGuideRatio2 * SPOOL_SPEED_OTHERS;
      guideAccelOthers = effectiveGuideRatio2 * SPOOL_ACCEL_OTHERS;
      guideMotor.setMaxSpeed(guideSpeedOthers);
      guideMotor.setAcceleration(guideAccelOthers);
      
      // Before fully starting the next layer, we retract the guide
      isRetractingGuide = true;
      guideMotor.move(-GUIDE_RETRACTION_STEPS);
      return;
    }
  }

  // For layer 1 only: re-zero the spool motor
  spoolMotor.setCurrentPosition(0);

  // Compute the guide motor's move relative to its start position
  currentGuideTarget = (long)(currentSpoolSteps * currentEffectiveGuideRatio);
  long targetSpool = currentSpoolSteps;
  long targetGuide = startGuidePosition + currentGuideDirection * currentGuideTarget;

  spoolMotor.moveTo(targetSpool);
  guideMotor.moveTo(targetGuide);

  finalDecelerationSet = false;
}

// --------------------------------------------------------
//   Setup
// --------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println("Type 'run' to start coil winding, 'pause' to pause,");
  Serial.println("'start' to resume, 'stop' to halt.");
  Serial.println("After layer 1, use 'adjust <offset>' to shift guide position, then 'continue'.");

  // Initialize spool for first layer speeds
  spoolMotor.setMaxSpeed(SPOOL_SPEED_LAYER1);
  spoolMotor.setAcceleration(SPOOL_ACCEL_LAYER1);

  // We'll dynamically set the guide speeds in startNewLayer
  guideMotor.setMaxSpeed(guideSpeedLayer1);
  guideMotor.setAcceleration(guideAccelLayer1);
}

// --------------------------------------------------------
//   Main Loop
// --------------------------------------------------------
void loop() {
  // Process serial commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("run")) {
      // Only start if not already running
      if (!isRunning) {
        Serial.println("Starting coil winding process...");
        isRunning    = true;
        currentLayer = 0;
        startNewLayer();
      }
    }
    else if (cmd.equalsIgnoreCase("pause")) {
      if (isRunning && !paused) {
        paused = true;
        // Save the remaining distance for each motor
        spoolRemaining = spoolMotor.distanceToGo();
        guideRemaining = guideMotor.distanceToGo();
        // Command the motors to decelerate to a stop
        spoolMotor.stop();
        guideMotor.stop();
        Serial.println("Winding process paused.");
      }
    }
    else if (cmd.equalsIgnoreCase("start")) { // 'start' resumes from pause
      if (isRunning && paused) {
        paused = false;
        // Reapply the proper acceleration based on the current layer
        if (currentLayer == 1) {
          spoolMotor.setAcceleration(SPOOL_ACCEL_LAYER1);
          guideMotor.setAcceleration(guideAccelLayer1);
        } else {
          spoolMotor.setAcceleration(SPOOL_ACCEL_OTHERS);
          guideMotor.setAcceleration(guideAccelOthers);
        }
        // Re-issue the move commands based on where we paused
        spoolMotor.moveTo(spoolMotor.currentPosition() + spoolRemaining);
        guideMotor.moveTo(guideMotor.currentPosition() + guideRemaining);
        Serial.println("Resuming winding process...");
      }
    }
    else if (cmd.equalsIgnoreCase("stop")) {
      Serial.println("Stopping winding process...");
      spoolMotor.stop();
      guideMotor.stop();
      isRunning = false;
      paused = false;
    }
    else if (cmd.startsWith("adjust")) {
      if (waitingForAdjustment) {
        // Expecting command format: "adjust <offset>"
        int offset = cmd.substring(7).toInt(); // extract offset value
        // Calculate new target position for the guide motor:
        long newPos = guideMotor.currentPosition() + offset;
        guideMotor.moveTo(newPos);
        Serial.print("Guide motor moving to new position (offset ");
        Serial.print(offset);
        Serial.println(").");
        // Update startGuidePosition so that layer 2 starts from here.
        startGuidePosition = newPos;
      } else {
        Serial.println("Adjustment not allowed at this time.");
      }
    }
    else if (cmd.equalsIgnoreCase("continue")) {
      if (waitingForAdjustment) {
        waitingForAdjustment = false;
        Serial.println("Continuing to layer 2...");
        continueToLayer2();
      } else {
        Serial.println("No adjustment pending.");
      }
    }
  }

  // If paused, do not run any motors until resumed
  if (paused) {
    return;
  }

  // If waiting for adjustment, run the guide motor so it reaches the new target
  if (waitingForAdjustment) {
    guideMotor.run();
    return;
  }

  // Handle guide retraction (for layers > 2)
  if (isRetractingGuide) {
    guideMotor.run();
    if (guideMotor.distanceToGo() == 0) {
      isRetractingGuide = false;
      Serial.println("Guide retraction complete. Starting next layer.");

      // For subsequent layers beyond 2, reverse guide direction
      startGuidePosition         = guideMotor.currentPosition();
      currentGuideDirection     *= -1;
      currentSpoolSteps          = SUBSEQUENT_LAYER_SPOOL_STEPS;
      currentEffectiveGuideRatio = effectiveGuideRatio2;

      // Re-zero spool for the new layer
      spoolMotor.setCurrentPosition(0);

      spoolMotor.setMaxSpeed(SPOOL_SPEED_OTHERS);
      spoolMotor.setAcceleration(SPOOL_ACCEL_OTHERS);

      guideSpeedOthers = effectiveGuideRatio2 * SPOOL_SPEED_OTHERS;
      guideAccelOthers = effectiveGuideRatio2 * SPOOL_ACCEL_OTHERS;
      guideMotor.setMaxSpeed(guideSpeedOthers);
      guideMotor.setAcceleration(guideAccelOthers);

      // Calculate new targets
      currentGuideTarget = (long)(currentSpoolSteps * currentEffectiveGuideRatio);
      long targetSpool    = currentSpoolSteps;
      long targetGuide    = startGuidePosition + currentGuideDirection * currentGuideTarget;

      spoolMotor.moveTo(targetSpool);
      guideMotor.moveTo(targetGuide);

      finalDecelerationSet = false;
    }
    return; // Wait until retraction completes
  }

  // If winding is active, run both motors
  if (isRunning) {
    spoolMotor.run();
    guideMotor.run();

    // -------------------------------------------------------
    // FINAL DECELERATION NEAR END
    // -------------------------------------------------------
    if (!finalDecelerationSet) {
      if (labs(guideMotor.distanceToGo()) <= GUIDE_FINAL_THRESHOLD) {
        guideMotor.setAcceleration(FINAL_ACCEL);
        Serial.println("Guide motor: Final deceleration engaged for stability (layer end).");
      }
      if (labs(spoolMotor.distanceToGo()) <= SPOOL_FINAL_THRESHOLD) {
        spoolMotor.setAcceleration(FINAL_ACCEL);
        Serial.println("Spool motor: Final deceleration engaged for stability (layer end).");
      }
      if (labs(guideMotor.distanceToGo()) <= GUIDE_FINAL_THRESHOLD ||
          labs(spoolMotor.distanceToGo()) <= SPOOL_FINAL_THRESHOLD)
      {
        finalDecelerationSet = true;
      }
    }

    // -------------------------------------------------------
    // LAYER COMPLETION
    // -------------------------------------------------------
    if (spoolMotor.distanceToGo() == 0 && guideMotor.distanceToGo() == 0 &&
        !isRetractingGuide)
    {
      Serial.println("Layer complete. Guide motor remains at last position.");
      startNewLayer();
    }
  }

  // -------------------------------------------------------
  // Return to home after final layer
  // -------------------------------------------------------
  if (!isRunning && currentLayer == TOTAL_LAYERS &&
      spoolMotor.distanceToGo() == 0 && guideMotor.distanceToGo() == 0)
  {
    Serial.println("Returned to home position. Winding process completed.");
    currentLayer++; // Prevent re-entering this block
  }
}

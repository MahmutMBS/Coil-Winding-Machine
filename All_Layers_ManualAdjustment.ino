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
const long FIRST_LAYER_SPOOL_STEPS      = 6000;  // spool steps for the first layer
const long SUBSEQUENT_LAYER_SPOOL_STEPS = 6000;  // spool steps for subsequent layers

// Base guide target in steps (input)
// This value will be the actual step count the guide motor moves per layer.
const long BASE_GUIDE_TARGET = 13600;

// Compute effective ratio from the given steps (for speed/acceleration only)
//const float effectiveGuideRatio1 = (float)BASE_GUIDE_TARGET / FIRST_LAYER_SPOOL_STEPS;
//const float effectiveGuideRatio2 = (float)BASE_GUIDE_TARGET / SUBSEQUENT_LAYER_SPOOL_STEPS;
const float effectiveGuideRatio1 = 2.35;
const float effectiveGuideRatio2 = 2.35;

// --------------------------------------------------------
//   Speeds & Accelerations
// --------------------------------------------------------
// FIRST layer spool speeds/accels
const float SPOOL_SPEED_LAYER1  = 500.0;  // steps/sec
const float SPOOL_ACCEL_LAYER1  = 300.0;  // steps/sec^2

// SUBSEQUENT layers spool speeds/accels
const float SPOOL_SPEED_OTHERS  = 500.0;  // steps/sec
const float SPOOL_ACCEL_OTHERS  = 300.0;  // steps/sec^2

// Guide motor speeds/accelerations (calculated as ratio * spool speed/accel)
float guideSpeedLayer1 = 0.0;
float guideAccelLayer1 = 0.0;
float guideSpeedOthers = 0.0;
float guideAccelOthers = 0.0;

// --------------------------------------------------------
//   Other Winding Settings
// --------------------------------------------------------
const long GUIDE_RETRACTION_STEPS = 0;  // (not used in adjustment mode)
const int  TOTAL_LAYERS           = 7;

// --------------------------------------------------------
//   State Variables
// --------------------------------------------------------
bool isRunning            = false;
int  currentLayer         = 0;

// Pause state variables
bool paused         = false;
long spoolRemaining = 0;
long guideRemaining = 0;

// NEW: Adjustment mode flag (applies to all layers except layer 1)
bool waitingForAdjustment = false;

// Variables for the current layer
long currentSpoolSteps          = 0;
long currentGuideTarget         = 0; // This will always equal BASE_GUIDE_TARGET
int  currentGuideDirection      = 1;   // 1 = forward, -1 = reverse
long startGuidePosition         = 0;   // starting position for guide motor for this layer
float currentEffectiveGuideRatio = 0.0;

// --------------------------------------------------------
//   Function to Continue to Next Layer After Adjustment
// --------------------------------------------------------
void continueToNextLayer() {
  // Set parameters for subsequent layers
  spoolMotor.setMaxSpeed(SPOOL_SPEED_OTHERS);
  spoolMotor.setAcceleration(SPOOL_ACCEL_OTHERS);
  
  guideSpeedOthers = effectiveGuideRatio2 * SPOOL_SPEED_OTHERS;
  guideAccelOthers = effectiveGuideRatio2 * SPOOL_ACCEL_OTHERS;
  guideMotor.setMaxSpeed(guideSpeedOthers);
  guideMotor.setAcceleration(guideAccelOthers);

  // Reverse guide direction (alternate each layer)
  currentGuideDirection *= -1;
  currentSpoolSteps = SUBSEQUENT_LAYER_SPOOL_STEPS;
  currentEffectiveGuideRatio = effectiveGuideRatio2;

  // Re-zero spool motor for the new layer
  spoolMotor.setCurrentPosition(0);

  // Reset the guide motor's relative position for the new layer
  guideMotor.setCurrentPosition(0);
  startGuidePosition = 0;

  // Use the constant target step count for the guide motor relative to 0
  currentGuideTarget = BASE_GUIDE_TARGET;
  long targetSpool = currentSpoolSteps;
  long targetGuide = currentGuideDirection * currentGuideTarget;

  spoolMotor.moveTo(targetSpool);
  guideMotor.moveTo(targetGuide);

  waitingForAdjustment = false;
  Serial.println("Continuing to next layer...");
}

// --------------------------------------------------------
//   Function to Start a New Layer
// --------------------------------------------------------
void startNewLayer() {
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
    // FIRST Layer: immediately start without adjustment
    spoolMotor.setMaxSpeed(SPOOL_SPEED_LAYER1);
    spoolMotor.setAcceleration(SPOOL_ACCEL_LAYER1);

    guideSpeedLayer1 = effectiveGuideRatio1 * SPOOL_SPEED_LAYER1;
    guideAccelLayer1 = effectiveGuideRatio1 * SPOOL_ACCEL_LAYER1;
    guideMotor.setMaxSpeed(guideSpeedLayer1);
    guideMotor.setAcceleration(guideAccelLayer1);

    // Reset guide motor's relative position for a clean start
    guideMotor.setCurrentPosition(0);
    startGuidePosition = 0;
    currentGuideDirection = 1;
    currentSpoolSteps = FIRST_LAYER_SPOOL_STEPS;
    currentEffectiveGuideRatio = effectiveGuideRatio1;
    
    // Re-zero spool motor
    spoolMotor.setCurrentPosition(0);
    
    // Use BASE_GUIDE_TARGET directly for the guide motor's target position
    currentGuideTarget = BASE_GUIDE_TARGET;
    long targetSpool = currentSpoolSteps;
    long targetGuide = currentGuideDirection * currentGuideTarget;
    
    spoolMotor.moveTo(targetSpool);
    guideMotor.moveTo(targetGuide);
  } else {
    // For layers 2 and beyond, enter adjustment mode.
    waitingForAdjustment = true;
    Serial.println("Layer complete.");
    Serial.println("Enter 'adjust <offset>' (e.g., 'adjust 100' or 'adjust -50') to shift guide position,");
    Serial.println("then type 'continue' to start the next layer.");
  }
}

// --------------------------------------------------------
//   Setup
// --------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println("Type 'run' to start coil winding, 'pause' to pause,");
  Serial.println("'start' to resume, 'stop' to halt.");
  Serial.println("For layers 2 and up, use 'adjust <offset>' to shift guide position, then 'continue'.");

  spoolMotor.setMaxSpeed(SPOOL_SPEED_LAYER1);
  spoolMotor.setAcceleration(SPOOL_ACCEL_LAYER1);

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
      if (!isRunning) {
        Serial.println("Starting coil winding process...");
        isRunning = true;
        currentLayer = 0;
        startNewLayer();
      }
    }
    else if (cmd.equalsIgnoreCase("pause")) {
      if (isRunning && !paused) {
        paused = true;
        spoolRemaining = spoolMotor.distanceToGo();
        guideRemaining = guideMotor.distanceToGo();
        spoolMotor.stop();
        guideMotor.stop();
        Serial.println("Winding process paused.");
      }
    }
    else if (cmd.equalsIgnoreCase("start")) {
      if (isRunning && paused) {
        paused = false;
        if (currentLayer == 1) {
          spoolMotor.setAcceleration(SPOOL_ACCEL_LAYER1);
          guideMotor.setAcceleration(guideAccelLayer1);
        } else {
          spoolMotor.setAcceleration(SPOOL_ACCEL_OTHERS);
          guideMotor.setAcceleration(guideAccelOthers);
        }
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
        // Expecting command: "adjust <offset>"
        int offset = cmd.substring(7).toInt();
        long newTarget = guideMotor.currentPosition() + offset;
        guideMotor.moveTo(newTarget);
        startGuidePosition = newTarget;
        Serial.print("Guide motor adjusted by ");
        Serial.print(offset);
        Serial.println(" steps.");
      } else {
        Serial.println("Adjustment not allowed at this time.");
      }
    }
    else if (cmd.equalsIgnoreCase("continue")) {
      if (waitingForAdjustment) {
        continueToNextLayer();
      } else {
        Serial.println("No adjustment pending.");
      }
    }
  }

  // If paused, do not run any motors
  if (paused) {
    return;
  }

  // If waiting for adjustment, run the guide motor to move to the new target
  if (waitingForAdjustment) {
    guideMotor.run();
    return;
  }

  // Normal operation: run both motors
  if (isRunning) {
    spoolMotor.run();
    guideMotor.run();

    // LAYER COMPLETION: when both motors reach target and not in adjustment
    if (spoolMotor.distanceToGo() == 0 && guideMotor.distanceToGo() == 0) {
      Serial.println("Layer complete. Guide motor remains at last position.");
      startNewLayer();
    }
  }

  // Return to home after final layer (if needed)
  if (!isRunning && currentLayer == TOTAL_LAYERS &&
      spoolMotor.distanceToGo() == 0 && guideMotor.distanceToGo() == 0) {
    Serial.println("Returned to home position. Winding process completed.");
    currentLayer++; // Prevent re-entry
  }
}

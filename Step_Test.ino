// --- Pin Assignments ---
#define DIR_PIN1   1   // Motor 1 DIR
#define STEP_PIN1  2   // Motor 1 STEP
#define DIR_PIN2   5   // Motor 2 DIR
#define STEP_PIN2  4   // Motor 2 STEP

void setup() {
  // --- Set the pins as outputs ---
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  
  // --- Set motor directions (HIGH or LOW) ---
  digitalWrite(DIR_PIN1, HIGH);  // Change to LOW to reverse Motor 1
  digitalWrite(DIR_PIN2, HIGH);  // Change to LOW to reverse Motor 2

  // If you have ENABLE pins, tie them appropriately (usually LOW to enable).
  // Also ensure your microstepping mode pins (M0, M1, M2 on DRV8825) are set as desired.
}

void loop() {
  // --- Step both motors simultaneously (same speed) ---
  digitalWrite(STEP_PIN1, HIGH);
  digitalWrite(STEP_PIN2, HIGH);
  delayMicroseconds(1500);  // Pulse width; adjust for motor requirements
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);
  delayMicroseconds(1500);  // Delay before the next step pulse

  // This loop runs continuously, causing both motors to step in sync.
}

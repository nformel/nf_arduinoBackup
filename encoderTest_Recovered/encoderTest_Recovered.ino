#include <RotaryEncoder.h>

const int SETTLE_DELAY_MS = 500;
const int STEP_DELAY_US = 1000;  // microseconds between steps
//const int TARGET_POSITION = 96;  // total counts (24-detent × 4)
const int TARGET_POSITION = 24;  // Doubled to compensate - encoder counts 2x faster than movement

//-------------------------------------------
// Pin definitions
//-------------------------------------------
#define stepperOn 33
#define stepperDir 27
#define stepperStep 28
#define stepperReset 29
#define stepperMode0 32
#define stepperMode1 31
#define stepperMode2 30
#define stepperEncAPin 10
#define stepperEncBPin 11
#define photoHomePin 15

//-------------------------------------------
// Encoder object
//-------------------------------------------
RotaryEncoder encoder(stepperEncAPin, stepperEncBPin, RotaryEncoder::LatchMode::FOUR3);

//-------------------------------------------
// Variables
//-------------------------------------------
int currentValveID = 0;
int nextValveID = 0;

//-------------------------------------------
// Setup
//-------------------------------------------
void setup() {
  Serial.begin(9600);
  delay(500);

  Serial.println("Stepper + Encoder Test Starting...");

  // Setup stepper pins
  pinMode(stepperOn, OUTPUT);
  pinMode(stepperDir, OUTPUT);
  pinMode(stepperStep, OUTPUT);
  pinMode(stepperReset, OUTPUT);
  pinMode(stepperMode0, OUTPUT);
  pinMode(stepperMode1, OUTPUT);
  pinMode(stepperMode2, OUTPUT);

  // Photogate
  pinMode(photoHomePin, INPUT);

  // Initialize stepper
  digitalWrite(stepperReset, HIGH);
  digitalWrite(stepperMode0, LOW);
  digitalWrite(stepperMode1, LOW);
  digitalWrite(stepperMode2, HIGH); // adjust microstepping if needed
  digitalWrite(stepperOn, LOW);

  // Encoder pins are automatically set up by the RotaryEncoder constructor
  // No begin() method needed for this library
  delay(500);
  
  Serial.println("Setup complete. Encoder initialized.");
}

//-------------------------------------------
// Stepper control to target position
//-------------------------------------------
void step_to_target(long target) {
  encoder.tick();  // Update encoder state
  long currentPos = encoder.getPosition();

  Serial.println("\n=== STEP_TO_TARGET DEBUG ===");
  Serial.println("Current position: " + String(currentPos));
  Serial.println("Target position: " + String(target));

  if (currentPos == target) {
    Serial.println("Already at target position!");
    return;
  }

  bool moveForward = (target > currentPos);
  Serial.println("Direction: " + String(moveForward ? "FORWARD" : "BACKWARD"));
  digitalWrite(stepperDir, moveForward ? HIGH : LOW);
  
  // Power up stepper
  digitalWrite(stepperMode2, LOW);
  digitalWrite(stepperMode1, LOW);
  digitalWrite(stepperMode0, HIGH);
  digitalWrite(stepperOn, HIGH);
  
  int stepCount = 0;

  while (true) {
    encoder.tick();  // CRITICAL: Poll encoder every loop iteration
    long pos = encoder.getPosition();

    if (stepCount % 50 == 0) {
      Serial.println("Steps: " + String(stepCount) + " | Encoder: " + String(pos) + " | Target: " + String(target));
    }

    if (pos == target) {
      Serial.println("TARGET REACHED! Steps: " + String(stepCount));
      break;
    }
    if ((moveForward && pos > target) || (!moveForward && pos < target)) {
      Serial.println("OVERSHOT! Steps: " + String(stepCount));
      break;
    }
    if (stepCount > 5000) {
      Serial.println("TIMEOUT! Too many steps without reaching target.");
      break;
    }

    // Step the motor
    digitalWrite(stepperStep, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepperStep, LOW);
    delayMicroseconds(STEP_DELAY_US);
    stepCount++;
  }

  Serial.println("Final position: " + String(encoder.getPosition()));
  
  // Power down stepper
  digitalWrite(stepperOn, LOW);
  delay(100);
}

//-------------------------------------------
// Zeroing function using photogate
//-------------------------------------------
void zeroValveNow() {
  Serial.println("Starting zeroing process...");
  digitalWrite(stepperReset, HIGH);
  digitalWrite(stepperMode2, LOW);
  digitalWrite(stepperMode1, LOW);
  digitalWrite(stepperMode0, HIGH);
  digitalWrite(stepperOn, HIGH);
  delay(50);
  digitalWrite(stepperDir, LOW);  // Move toward home

  int stepCount = 0;
  int signalCount = 0;
  int lastState = digitalRead(photoHomePin);
  Serial.print("Photo Sensor Initial State: "); Serial.println(lastState);

  while (signalCount < 7) {  // Stop on 7th HIGH signal
    // Poll encoder during zeroing too
    encoder.tick();
    
    digitalWrite(stepperStep, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepperStep, LOW);
    delayMicroseconds(STEP_DELAY_US);
    stepCount++;

    int currentState = digitalRead(photoHomePin);
    if (currentState == 1) {
      signalCount++;
    }
  }

  Serial.println("7th photogate trigger reached. Stopping motor.");
  Serial.print("Total steps to zero: "); Serial.println(stepCount);

  // Reset encoder
  encoder.setPosition(0);

  // Power down stepper
  digitalWrite(stepperMode2, LOW);
  digitalWrite(stepperMode1, LOW);
  digitalWrite(stepperMode0, LOW);
  digitalWrite(stepperOn, LOW);
}

//-------------------------------------------
// Valve ID → absolute encoder target
//-------------------------------------------
long valve_to_absolute_target(int valveID, int numValves=16) {
  // Use float math and round properly
  float position = (valveID - 1) * (TARGET_POSITION / (float)numValves);
  return (long)round(position);  // Use round() function
}

//-------------------------------------------
// Main loop
//-------------------------------------------
// void loop() {
//   int numValves = 16;

//   zeroValveNow();
//   currentValveID = 1;
//   delay(5000);

//   int testValves[] = {3,4,5,6,7,8,9,10,11,12,13,14,15,16};
//   int numTestValves = sizeof(testValves) / sizeof(testValves[0]);

//   for (int v = 0; v < numTestValves; v++) {
//     int newValveID = testValves[v];

//     Serial.println("\n########## STARTING SEQUENCE FOR VALVE #" + String(newValveID) + " ##########");

//     long target = valve_to_absolute_target(newValveID);
//     step_to_target(target);
//     currentValveID = newValveID;

//     // Power down stepper driver
//     digitalWrite(stepperOn, LOW);
//     digitalWrite(stepperMode2, LOW);
//     digitalWrite(stepperMode1, LOW);
//     digitalWrite(stepperMode0, LOW);
//     delay(10000);

//     Serial.println("########## SEQUENCE COMPLETE FOR VALVE #" + String(newValveID) + " ##########");
//   }

//   Serial.println("\nWaiting 10 minutes before next cycle...");
//   delay(600000);
// }


void loop() {
  int numValves = 16;
  currentValveID = 1;
  delay(5000);
  int testValves[] = {3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  int numTestValves = sizeof(testValves) / sizeof(testValves[0]);
  for (int v = 0; v < numTestValves; v++) {
      int newValveID = testValves[v];
      Serial.println("\n########## STARTING SEQUENCE FOR VALVE #" + String(newValveID) + " ##########");
      // Zero the system
      zeroValveNow();
      delay(5000);
      // Move to test valve
      Serial.println("Moving to valve #" + String(newValveID));
      long target = valve_to_absolute_target(newValveID);
      step_to_target(target);
      currentValveID = newValveID;
      // Power down stepper driver
      digitalWrite(stepperOn, LOW);
      digitalWrite(stepperMode2, LOW);
      digitalWrite(stepperMode1, LOW);
      digitalWrite(stepperMode0, LOW);
      delay(5000);
      // Move back to valve 2
      Serial.println("Moving back to valve #2");
      long target2 = valve_to_absolute_target(2);
      step_to_target(target2);
      currentValveID = 2;
      // Power down stepper driver
      digitalWrite(stepperOn, LOW);
      digitalWrite(stepperMode2, LOW);
      digitalWrite(stepperMode1, LOW);
      digitalWrite(stepperMode0, LOW);
      delay(5000);
      Serial.println("########## SEQUENCE COMPLETE FOR VALVE #" + String(newValveID) + " ##########");
    }

    Serial.println("\nWaiting 10 minutes before next cycle...");
    delay(600000);
  }

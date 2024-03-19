#include <AccelStepper.h>

// Define stepper motor connections
#define STEP_PIN 9
#define DIR_PIN 8

char input;
int targetPosition;

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(115200);

  // Set maximum speed and acceleration
  stepper.setMaxSpeed(1200); // Adjust as needed
  stepper.setAcceleration(850); // Adjust as needed
}

void loop() {
  if (Serial.available() > 0) {
    input = Serial.read(); // Read the input string from serial
    targetPosition = atoi(&input); // Convert the input string to an integer
    
    // Move the stepper motor to the specified position
    if (targetPosition > 0) {
      moveBackward(targetPosition);
    } else if (targetPosition < 0) {
//      moveForward(-targetPosition);
      moveForward(targetPosition);
    }
  }
}

void moveForward(int target) {
  digitalWrite(DIR_PIN, HIGH); // Set the direction to forward
  stepper.moveTo(target); // Move to the target position
  
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}

void moveBackward(int target) {
  digitalWrite(DIR_PIN, LOW); // Set the direction to backward
  stepper.moveTo(target); // Move to the target position
  
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}

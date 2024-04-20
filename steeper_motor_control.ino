#include <Stepper.h>
#include <Wire.h>    // Include the Wire library for I2C communication
#include <AS5600.h> // Include the AS5600 library

// Define the number of steps per revolution
const int stepsPerRevolution = 200;

// Initialize the AS5600 sensor
AS5600 as5600;

// Function to rotate a stepper motor to a specified angle
void rotateToAngle(float desiredAngle, int pulPin, int dirPin, int enaPin) {
  // Initialize the stepper motor with the number of steps and pin numbers
  Stepper myStepper(stepsPerRevolution, pulPin, dirPin, enaPin);

  // Calculate the number of steps needed to rotate to the desired angle
  int stepsToMove = (int)((desiredAngle / 360.0) * stepsPerRevolution);

  // Set the speed of the stepper motor
  myStepper.setSpeed(60); // Set to 60 RPM, adjust as needed

  // Rotate the stepper motor to the desired angle
  myStepper.step(stepsToMove);
}

void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(9600);

  // Initialize I2C communication
  Wire.begin();

  // Initialize AS5600 sensor
  as5600.begin();
}

void loop() {
  // Read the current angle from the AS5600 sensor
  float currentAngle = as5600.getAngle();

  // Print the current angle to Serial Monitor
  Serial.print("Current Angle: ");
  Serial.println(currentAngle);

  // Rotate the stepper motor to the current angle with pin connections (8, 9, 10)
  rotateToAngle(currentAngle, 8, 9, 10);

  // Add a delay to avoid reading the angle too frequently
  delay(1000); // Adjust delay as needed
}

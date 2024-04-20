#include <Servo.h>

// Create a servo object
Servo servoMotor;

void setup() {
  // No need to attach the servo here, as it will be attached dynamically
}

void loop() {
  // Move servo connected to pin 9 to 90 degrees slowly
  moveServo(9, 90, 2); // Move to 90 degrees with 2 degrees step size
  delay(1000); // Wait for 1 second
  
  // Move servo connected to pin 10 to 0 degrees slowly
  moveServo(10, 0, -2); // Move to 0 degrees with 2 degrees step size in reverse direction
  delay(1000); // Wait for 1 second
  
  // Move servo connected to pin 11 to 180 degrees slowly
  moveServo(11, 180, 2); // Move to 180 degrees with 2 degrees step size
  delay(1000); // Wait for 1 second
}

void moveServo(int servoPin, int targetAngle, int stepSize) {
  // Attach the servo to the specified pin
  servoMotor.attach(servoPin);

  int currentAngle = servoMotor.read();
  
  while (currentAngle != targetAngle) {
    if (currentAngle < targetAngle) {
      currentAngle += stepSize;
      if (currentAngle > targetAngle) {
        currentAngle = targetAngle;
      }
    } else {
      currentAngle -= stepSize;
      if (currentAngle < targetAngle) {
        currentAngle = targetAngle;
      }
    }
    servoMotor.write(currentAngle);
    delay(15); // Adjust this delay to control the speed of movement
  }

  // Detach the servo after movement is completed
  servoMotor.detach();
}

# Motor Control Arduino Repository

This repository contains Arduino code for controlling various motors and simulating a robotic arm. It includes implementations for stepper motor control, servo motor control, inverse kinematics for a 5-DOF manipulator, and a combined control system. Below is a brief description of each file in the repository.

## Files

### 1. `steeper_motor_control.ino`
This file demonstrates the control of a stepper motor using the `Stepper` library and an AS5600 sensor for position feedback. The main functions include:
- `rotateToAngle(float desiredAngle, int pulPin, int dirPin, int enaPin)`: Rotates the stepper motor to a specified angle.
- `setup()`: Initializes Serial communication, I2C, and the AS5600 sensor.
- `loop()`: Continuously reads the angle from the AS5600 sensor and rotates the stepper motor to that angle.

### 2. `servo_motor_control.ino`
This file controls a servo motor and demonstrates moving it to different angles with a specified step size. Key functions include:
- `moveServo(int servoPin, int targetAngle, int stepSize)`: Moves the servo to the target angle in increments defined by `stepSize`.
- `setup()`: No servo attachment here; it's done dynamically.
- `loop()`: Moves the servo to various angles (90°, 0°, and 180°) with delays in between.

### 3. `inverse_kinematics_5DOF_manipulator.ino`
This file simulates the movement of a 5-DOF robotic arm using inverse kinematics. It calculates joint angles based on desired end effector positions and updates these angles over time. It includes:
- `rad2deg(float rad)`: Converts radians to degrees.
- `RoboticArmjointAngle(float h, int t_final, float l1, float l2, ...)`: Simulates the movement of the robotic arm and computes the final joint angles.
- `setup()`: Initializes Serial communication.
- `loop()`: Calls the simulation function and prints the final joint angles to the Serial Monitor.

### 4. `combined.ino`
This file combines functionalities from the previous code files. It:
- Calculates joint angles for a 5-DOF manipulator using inverse kinematics.
- Reads the angle from an AS5600 sensor.
- Controls stepper motors and servo motors based on the calculated joint angles.
- Includes functions like `moveServo(int servoPin, float targetAngle, int stepSize)`, `StepperRotateToAngle(float desiredAngle, int pulPin, int dirPin, int enaPin)`, and `RoboticArmjointAngle(...)`.
- Contains `setup()` and `loop()` functions to initialize communication and execute the combined control logic.

## Usage

1. **Install Libraries**: Ensure you have the necessary Arduino libraries installed:
   - `Stepper`
   - `Wire`
   - `AS5600`
   - `Servo`

2. **Connect Hardware**: Wire your stepper motors, servo motors, and AS5600 sensor to the appropriate pins as defined in the code.

3. **Upload Code**: Upload the desired `.ino` file to your Arduino board using the Arduino IDE.

4. **Monitor Output**: Use the Serial Monitor to view debugging information, including joint angles and motor positions.

## Notes

- Ensure that pin numbers in the code match your hardware connections.
- Adjust parameters such as step sizes, delays, and initial angles based on your specific application needs.

Feel free to modify and extend the code to suit your project requirements. For any issues or questions, please check the documentation or seek help from the Arduino community.

Happy coding!

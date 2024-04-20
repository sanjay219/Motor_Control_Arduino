#include <Arduino.h>
#include <math.h>
#include <Stepper.h>
#include <Wire.h> 
#include <AS5600.h>
#include <Servo.h>

// Create a servo object
Servo servoMotor;

// Define the number of steps per revolution
const int stepsPerRevolution = 200;

// Initialize the AS5600 sensor
AS5600 as5600;

// Function to convert radians to degrees
float rad2deg(float rad)
{
    return rad * 180.0 / PI;
}

float *RoboticArmjointAngle(float h, int t_final, float l1, float l2, float th4_initial, float th5_initial, float qx_initial, float qy_initial, float qz_initial)
{
    // Initialize variables
    float th1, th2, th3, th4, th5;
    float qx = qx_initial;
    float qy = qy_initial;
    float qz = qz_initial;

    // Array to store final joint angles
    static float finalAngles[5];

    // Loop until end effector reaches desired position or time expires
    for (int iter = 0; iter * h <= t_final; iter++)
    {
        // Calculate initial joint angles
        th1 = atan2(qy, qx);
        th3 = acos((qx * qx + qy * qy + qz * qz - l1 * l1 - l2 * l2) / (2 * l1 * l2));
        th2 = atan2((l1 + l2 * cos(th3)), (l2 * sin(th3))) + atan2((qx * cos(th1) + qy * sin(th1)), qz);

        float r11 = cos(th1) * cos(th2 + th3 + th4) * cos(th5) - sin(th1) * sin(th5);
        float r12 = -cos(th1) * cos(th2 + th3 + th4) * sin(th5) - sin(th1) * cos(th5);
        float r13 = cos(th1) * sin(th2 + th3 + th4);
        float r21 = sin(th1) * cos(th2 + th3 + th4) * cos(th5) + cos(th1) * sin(th5);
        float r22 = -sin(th1) * cos(th2 + th3 + th4) * sin(th5) + cos(th1) * cos(th5);
        float r23 = sin(th1) * sin(th2 + th3 + th4);
        float r31 = -sin(th2 + th3 + th4) * cos(th5);
        float r32 = sin(th2 + th3 + th4) * sin(th5);
        float r33 = cos(th2 + th3 + th4);

        // Update end effector position
        qx += h / 6000;
        qy += h / 6000;
        qz += h / 6000;

        // Calculate new joint angles
        th1 = atan2(qy, qx);
        th2 = atan2((l1 + l2 * cos(th3)), (l2 * sin(th3))) + atan2((qx * cos(th1) + qy * sin(th1)), qz);
        th3 = acos((qx * qx + qy * qy + qz * qz - l1 * l1 - l2 * l2) / (2 * l1 * l2));
        th4 = atan2((r13 * cos(th1) + r23 * sin(th1)), r33) - th2 - th3;
        th5 = (r12 * cos(th1) - r11 * sin(th1)) / (r22 * cos(th1) - r21 * sin(th1));

	// below mentioned conditions are not correct we need to modify it as per our requirement************************************************************
        if (qx >= 10 && qy >= 10 && qz >= 10)
        {
            break; // End the loop if desired position is reached
        }

        // Delay to control simulation speed
        delay(h);
    }

    // Store final joint angles in the array
    finalAngles[0] = th1;
    finalAngles[1] = th2;
    finalAngles[2] = th3;
    finalAngles[3] = th4;
    finalAngles[4] = th5;

    // Return the array of final joint angles
    return finalAngles;
}

void moveServo(int servoPin, float targetAngle, int stepSize)
{
    // Attach the servo to the specified pin
    servoMotor.attach(servoPin);

    int currentAngle = servoMotor.read();

    while (currentAngle != targetAngle)
    {
        if (currentAngle < targetAngle)
        {
            currentAngle += stepSize;
            if (currentAngle > targetAngle)
            {
                currentAngle = targetAngle;
            }
        }
        else
        {
            currentAngle -= stepSize;
            if (currentAngle < targetAngle)
            {
                currentAngle = targetAngle;
            }
        }
        servoMotor.write(currentAngle);
        delay(15); // Adjust this delay to control the speed of movement*******************************************************
    }

    // Detach the servo after movement is completed
    servoMotor.detach();
}

// Function to rotate a stepper motor to a specified angle
void StepperRotateToAngle(float desiredAngle, int pulPin, int dirPin, int enaPin)
{
    // Initialize the stepper motor with the number of steps and pin numbers
    Stepper myStepper(stepsPerRevolution, pulPin, dirPin, enaPin);

    // Calculate the number of steps needed to rotate to the desired angle
    int stepsToMove = (int)((desiredAngle / 360.0) * stepsPerRevolution);

    // Set the speed of the stepper motor
    myStepper.setSpeed(60); // Set to 60 RPM, adjust as needed

    // Rotate the stepper motor to the desired angle
    myStepper.step(stepsToMove);
}

void setup()
{
    // Initialize Serial communication for debugging
    Serial.begin(9600);

    // Initialize I2C communication
    Wire.begin();

    // Initialize AS5600 sensor
    as5600.begin();
}

void loop()
{
    //the pin number provided with the functions are not correct please change them as per the coonection***************************************

    // Parameters for RoboticArmjointAngle function
    float h = 0.001;     // Step size
    int t_final = 20000; // Final time in milliseconds (20 seconds)
    float l1 = 0.22839;  // Length of link 1 in m
    float l2 = 0.248;    // Length of link 2 in m

    float th4_initial = 0.17; // initial 10 degrees
    float th5_initial = 0.26; // initial 15 degrees

    float qx_initial = 0.001; // initial X-coordinate of the end effector in m
    float qy_initial = 0.001; // initial Y-coordinate of the end effector in m
    float qz_initial = 0.001; // initial Z-coordinate of the end effector in m

    // Call the robotic arm joint angle function
    float *finalJointAngles = RoboticArmjointAngle(h, t_final, l1, l2, th4_initial, th5_initial, qx_initial, qy_initial, qz_initial);

    float th1_final = finalJointAngles[0];
    float th2_final = finalJointAngles[1];
    float th3_final = finalJointAngles[2];
    float th4_final = finalJointAngles[3];
    float th5_final = finalJointAngles[4];

    // Read the current angle from the AS5600 sensor
    // as5600.getAngle() will be used in the latter phase 
    // below three lines are used to check whether the sensors are working or not*****************************************

    float currentAngle = as5600.getAngle();
    // Print the current angle to Serial Monitor
    Serial.print("Current Angle: ");
    Serial.println(currentAngle);

    // need to check whether the servo is moving properly or not otherwise ************************************
    // we need to use rad2deg *********************************************************************************

    // Rotate the stepper motor to the current angle with pin connections (8, 9, 10)
    StepperRotateToAngle(th1_final, 8, 9, 10);
    
    StepperRotateToAngle(th2_final, 11, 10, 15);

    // Add a delay as needed to avoid reading the angle too frequently
    delay(1000); 
    //----------------------------------------------------------------------

    // Move servo connected to pin 9 to th3_final degrees slowly

    moveServo(9, th3_final , 2); 
    delay(1000);         // Wait for 1 second

    // Move servo connected to pin 10 to th4_final degrees slowly
    moveServo(10, th4_final, 2); 
    delay(1000);          

    // Move servo connected to pin 11 to th5_final degrees slowly
    moveServo(11, th5_final, 2); 
    delay(1000);           

    // Print the final joint angles
    Serial.println("Final Joint Angles:");
    Serial.print("th1: ");
    Serial.println(rad2deg(finalAngles[0]));
    Serial.print("th2: ");
    Serial.println(rad2deg(finalAngles[1]));
    Serial.print("th3: ");
    Serial.println(rad2deg(finalAngles[2]));
    Serial.print("th4: ");
    Serial.println(rad2deg(finalAngles[3]));
    Serial.print("th5: ");
    Serial.println(rad2deg(finalAngles[4]));

    // Uncomment the below line if you want the simulation to run continuously
    // while (true);
}
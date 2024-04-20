#include <Arduino.h>
#include <math.h>

// Function to convert radians to degrees
float rad2deg(float rad) {
  return rad * 180.0 / PI;
}

// Function to simulate the movement of a robotic arm
float* simulateRoboticArm(float h, int t_final, float l1, float l2, float th4_initial, float th5_initial, float qx_initial, float qy_initial, float qz_initial) {
  // Initialize variables
  float th1, th2, th3, th4, th5;
  float qx = qx_initial;
  float qy = qy_initial;
  float qz = qz_initial;

  // Array to store final joint angles
  static float finalAngles[5];

  // Loop until end effector reaches desired position or time expires
  for (int iter = 0; iter * h <= t_final; iter++) {
    // Calculate initial joint angles
    th1 = atan2(qy, qx);
    th3 = acos((qx * qx + qy * qy + qz * qz - l1 * l1 - l2 * l2) / (2 * l1 * l2));
    th2 = atan2((l1 + l2 * cos(th3)), (l2 * sin(th3))) + atan2((qx * cos(th1) + qy * sin(th1)), qz);

    // Calculate rotation matrix elements
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
        // th4 = atan2((r13 * cos(th1) + r23 * sin(th1)), r33) - th2 - th3;
    th4 = atan2((r13 * cos(th1) + r23 * sin(th1)), r33) - th2 - th3;
    th5 = (r12 * cos(th1) - r11 * sin(th1)) / (r22 * cos(th1) - r21 * sin(th1));

    // Check if end effector reaches the desired position
    if (qx >= 10 && qy >= 10 && qz >= 10) {
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

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Parameters for simulateRoboticArm function
  float h = 0.001; // Step size
  int t_final = 20000; // Final time in milliseconds (20 seconds)
  float l1 = 0.22839; // Length of link 1
  float l2 = 0.248;   // Length of link 2
  float th4_initial = 0.17; // 10 degrees
  float th5_initial = 0.26; // 15 degrees
  float qx_initial = 0.001; // X-coordinate of the end effector
  float qy_initial = 0.001; // Y-coordinate of the end effector
  float qz_initial = 0.001; // Z-coordinate of the end effector

  // Call the robotic arm simulation function
  float* finalAngles = simulateRoboticArm(h, t_final, l1, l2, th4_initial, th5_initial, qx_initial, qy_initial, qz_initial);

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


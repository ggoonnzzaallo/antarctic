#include <Servo.h>

Servo servo1;  // create servo object for servo1, this is the knee
Servo servo2;  // create servo object for servo2, this is the hip

// Define possible angles for Servo 2
int servo2Angles[] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85};

// Updated minimum and maximum angles for Servo 1
float servo1MinAngles[] = {85, 85, 85, 85, 85, 85, 100, 100, 110, 120, 125, 130, 135, 140, 145, 150, 155, 165};
float servo1MaxAngles[] = {170, 170, 170, 170, 170, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175};

void setup() {
  servo1.attach(9);  // Attach servo1 to pin 9
  servo2.attach(10); // Attach servo2 to pin 10
  Serial.begin(9600); // Start Serial communication for debugging
}

void loop() {
  // Sweep through Servo 2 angles with Servo 1 minimum angles
  for (int i = 0; i < 18; i++) {
    servo2.write(servo2Angles[i]);     // Set Servo 2 to the current angle
    servo1.write(servo1MinAngles[i]);  // Set Servo 1 to the corresponding minimum angle
    Serial.print("Servo 2 Angle: ");
    Serial.print(servo2Angles[i]);
    Serial.print(" | Servo 1 Min Angle: ");
    Serial.println(servo1MinAngles[i]);
    delay(20);
  }

  // Sweep through Servo 2 angles with Servo 1 maximum angles
  for (int i = 0; i < 18; i++) {
    servo2.write(servo2Angles[i]);     // Set Servo 2 to the current angle
    servo1.write(servo1MaxAngles[i]);  // Set Servo 1 to the corresponding maximum angle
    Serial.print("Servo 2 Angle: ");
    Serial.print(servo2Angles[i]);
    Serial.print(" | Servo 1 Max Angle: ");
    Serial.println(servo1MaxAngles[i]);
    delay(20);
  }
}

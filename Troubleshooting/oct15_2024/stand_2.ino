#include <Servo.h>

Servo servo1;  // RR Shoulder
Servo servo2;  // RR Hip
Servo servo3;  // RR Knee
Servo servo4;  // LR Shoulder
Servo servo5;  // LR Hip
Servo servo6;  // LR Knee
Servo servo7;  // FR Shoulder
Servo servo8;  // FR Hip
Servo servo9;  // FR Knee
Servo servo10; // FL Shoulder
Servo servo11; // FL Hip
Servo servo12; // FL Knee

void setup() {
  // Attach each servo to its corresponding pin
  servo1.attach(1);   // RR Shoulder
  servo2.attach(2);   // RR Hip
  servo3.attach(3);   // RR Knee
  servo4.attach(4);   // LR Shoulder
  servo5.attach(5);   // LR Hip
  servo6.attach(6);   // LR Knee
  servo7.attach(7);   // FR Shoulder
  servo8.attach(8);   // FR Hip
  servo9.attach(9);   // FR Knee
  servo10.attach(10); // FL Shoulder
  servo11.attach(11); // FL Hip
  servo12.attach(12); // FL Knee

  // Set servo positions
  servo1.write(90);   // RR Shoulder
  servo2.write(30);   // RR Hip
  servo3.write(140);  // RR Knee
  servo4.write(90);   // LR Shoulder
  servo5.write(150);  // LR Hip
  servo6.write(40);   // LR Knee
  servo7.write(90);   // FR Shoulder
  servo8.write(30);   // FR Hip
  servo9.write(140);  // FR Knee
  servo10.write(90);  // FL Shoulder
  servo11.write(150); // FL Hip
  servo12.write(40);  // FL Knee
}

void loop() {
  // The servos will hold their positions
}


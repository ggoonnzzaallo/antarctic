#include <Servo.h>

// This is code to control a quadruped robot.
// The left servos are mirrored with respect to the right servos.
// An angle of 0 for the hip servos on the right is equivalent to an agel of 180 for the servos on the left. The range for all servos is 0 to 180. 0 on the right is equivalent to 180 on the left (flat)
// An angel of 90 for the servo knee on the right is the same as the servo knee on the left, but 100 on the right is equivalent to 80 on the left.

Servo FLKnee;  // Create servo objects
Servo FLHip;
Servo FLShoulder;
Servo FRKnee;
Servo FRHip;
Servo FRShoulder;
Servo RLKnee;
Servo RLHip;
Servo RLShoulder;
Servo RRKnee;
Servo RRHip;
Servo RRShoulder;

// Trim constants (adjust these values as needed)
const int FLKnee_TRIM = 0;     // Example: -5 to +5
const int FLHip_TRIM = 0;
const int FRKnee_TRIM = 0;
const int FRHip_TRIM = 10; //10
const int RLKnee_TRIM = 0;
const int RLHip_TRIM = 0;
const int RRKnee_TRIM = 0;
const int RRHip_TRIM = 15; //20
const int FLShoulder_TRIM = 0;
const int FRShoulder_TRIM = 0;
const int RLShoulder_TRIM = 0;
const int RRShoulder_TRIM = 0;

// Standing position constants
const int STAND_RIGHT_HIP = 50;    // Standing position for right side
const int STAND_RIGHT_KNEE = 115;
const int STAND_LEFT_HIP = 120;    // Standing position for left side
const int STAND_LEFT_KNEE = 65;

bool isRunning = true;

void setup() {
    Serial.begin(9600);
    delay(1000);

    // Initialize shoulders to 90 degrees (default position)
    writeServoWithTrim(RRShoulder, 90, RRShoulder_TRIM);
    writeServoWithTrim(RLShoulder,90, RLShoulder_TRIM);
    writeServoWithTrim(FRShoulder, 90, FRShoulder_TRIM);
    writeServoWithTrim(FLShoulder, 90, FLShoulder_TRIM);

    RRShoulder.attach(A1);  // Rear Right Shoulder
    RRHip.attach(12); // Rear Right Hip
    RRKnee.attach(11); // Rear Right Knee

    RLShoulder.attach(10); // Rear Left Shoulder
    RLHip.attach(9); // Rear Left Hip
    RLKnee.attach(8); // Rear Left Knee

    FRShoulder.attach(7); // Front Right Shoulder
    FRHip.attach(5); // Front Right Hip
    FRKnee.attach(6); // Front Right Knee

    FLShoulder.attach(4); // Front Left Shoulder
    FLHip.attach(2); // Front Left Hip
    FLKnee.attach(3); // Front Left Knee

    // Initial standing position
    writeServoWithTrim(RRHip, STAND_RIGHT_HIP, RRHip_TRIM);
    writeServoWithTrim(RRKnee, STAND_RIGHT_KNEE, RRKnee_TRIM);
    writeServoWithTrim(FRHip, STAND_RIGHT_HIP, FRHip_TRIM);
    writeServoWithTrim(FRKnee, STAND_RIGHT_KNEE, FRKnee_TRIM);
    
    // Add left side standing position
    writeServoWithTrim(RLHip, STAND_LEFT_HIP, RLHip_TRIM);  
    writeServoWithTrim(RLKnee, STAND_LEFT_KNEE, RLKnee_TRIM);  
    writeServoWithTrim(FLHip, STAND_LEFT_HIP, FLHip_TRIM);
    writeServoWithTrim(FLKnee, STAND_LEFT_KNEE, FLKnee_TRIM);

    writeServoWithTrim(FRHip, STAND_RIGHT_HIP-30, FRHip_TRIM);

}

void writeServoWithTrim(Servo &servo, int angle, int trim) {
    servo.write(constrain(angle + trim, 0, 180));
}

void loop() {
    if(Serial.available() > 0) {
        String command = Serial.readString();
        if(command == "stop") {
            isRunning = false;
        } else if(command == "go") {
            isRunning = true;
        }
    }
    
  

    // if(isRunning) {  

    //     //FR Step forward
    //     writeServoWithTrim(FRHip, STAND_RIGHT_HIP - 5, FRHip_TRIM);
    //     writeServoWithTrim(FRKnee, STAND_RIGHT_KNEE + 5, FRKnee_TRIM);
    //     delay(100);
    //     writeServoWithTrim(FRHip, 60, FRHip_TRIM);
    //     writeServoWithTrim(FRKnee, 120, FRKnee_TRIM);
    //     delay(200);

    //     //RR Step forward
    //     writeServoWithTrim(RRHip, STAND_RIGHT_HIP - 5, RRHip_TRIM);
    //     writeServoWithTrim(RRKnee, STAND_RIGHT_KNEE + 5, RRKnee_TRIM);
    //     delay(200);
    //     writeServoWithTrim(RRHip, 60, RRHip_TRIM); //90
    //     writeServoWithTrim(RRKnee, 120, RRKnee_TRIM); //150
    //     delay(200);

    //     //RL Step forward 
    //     writeServoWithTrim(RLHip, STAND_LEFT_HIP + 10, RLHip_TRIM);
    //     writeServoWithTrim(RLKnee, STAND_LEFT_KNEE - 10, RLKnee_TRIM);
    //     delay(100);
    //     writeServoWithTrim(RLHip, 120, RLHip_TRIM);
    //     writeServoWithTrim(RLKnee, 60, RLKnee_TRIM);
    //     delay(200);

    //     //FL Step forward 
    //     writeServoWithTrim(FLHip, STAND_LEFT_HIP + 10, FLHip_TRIM);
    //     writeServoWithTrim(FLKnee, STAND_LEFT_KNEE - 10, FLKnee_TRIM);
    //     delay(100);
    //     writeServoWithTrim(FLHip, 120, FLHip_TRIM);
    //     writeServoWithTrim(FLKnee, 60, FLKnee_TRIM);
    //     delay(500);

    // //     //Everyone goes to stand
    //       writeServoWithTrim(FRHip, STAND_RIGHT_HIP, FRHip_TRIM);
    //       writeServoWithTrim(FRKnee, STAND_RIGHT_KNEE, FRKnee_TRIM);
    //       writeServoWithTrim(RRHip, STAND_RIGHT_HIP, RRHip_TRIM);
    //       writeServoWithTrim(RRKnee, STAND_RIGHT_KNEE, RRKnee_TRIM);
    //       writeServoWithTrim(FLHip, STAND_LEFT_HIP, FLHip_TRIM);
    //       writeServoWithTrim(FLKnee, STAND_LEFT_KNEE, FLKnee_TRIM);
    //       writeServoWithTrim(RLHip, STAND_LEFT_HIP, RLHip_TRIM);
    //       writeServoWithTrim(RLKnee, STAND_LEFT_KNEE, RLKnee_TRIM);

    //     delay(500);
    // }
}

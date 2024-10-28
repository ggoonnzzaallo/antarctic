#include <Servo.h>

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

bool isRunning = true;

void moveLegs(Servo& hipRight, Servo& kneeRight, Servo& hipLeft, Servo& kneeLeft,
              Servo& supportHipRight, Servo& supportKneeRight, 
              Servo& supportHipLeft, Servo& supportKneeLeft) {
    // Standing position
    const int RIGHT_HIP_STAND = 70;
    const int RIGHT_KNEE_STAND = 130;
    const int LEFT_HIP_STAND = 110;    // 180 - 70
    const int LEFT_KNEE_STAND = 50;    // 180 - 130

    // Movement angles
    const int RIGHT_HIP_LIFT = 90;
    const int RIGHT_KNEE_LIFT = 150;
    const int RIGHT_HIP_FORWARD = 40;
    const int RIGHT_KNEE_FORWARD = 70;
    
    const int LEFT_HIP_LIFT = 90;
    const int LEFT_KNEE_LIFT = 30;     // 180 - 150
    const int LEFT_HIP_FORWARD = 140;  // 180 - 40
    const int LEFT_KNEE_FORWARD = 110; // 180 - 70

    // Phase 1: First pair pushes while support pair maintains standing
    hipRight.write(35);  // Push position
    kneeRight.write(130);
    hipLeft.write(145);
    kneeLeft.write(50);
    
    supportHipRight.write(RIGHT_HIP_STAND);  // Standing position
    supportKneeRight.write(RIGHT_KNEE_STAND);
    supportHipLeft.write(LEFT_HIP_STAND);
    supportKneeLeft.write(LEFT_KNEE_STAND);
    delay(300);
    
    // Phase 2: Support pair quickly lifts and strides forward
    supportHipRight.write(RIGHT_HIP_LIFT);
    supportKneeRight.write(RIGHT_KNEE_LIFT);
    supportHipLeft.write(LEFT_HIP_LIFT);
    supportKneeLeft.write(LEFT_KNEE_LIFT);
    delay(100);
    
    supportHipRight.write(RIGHT_HIP_FORWARD);
    supportKneeRight.write(RIGHT_KNEE_FORWARD);
    supportHipLeft.write(LEFT_HIP_FORWARD);
    supportKneeLeft.write(LEFT_KNEE_FORWARD);
    delay(200);
}

void setup() {
    Serial.begin(9600);
    delay(1000);

    // Initialize shoulders to 90 degrees (default position)
    RRShoulder.write(90);
    RLShoulder.write(90);
    FRShoulder.write(90);
    FLShoulder.write(90);

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
    RRHip.write(70);
    RRKnee.write(130);
    FRHip.write(70);
    FRKnee.write(130);
    
    // Add left side standing position
    RLHip.write(110);  // 180° - 70° = 110°
    RLKnee.write(50);  // 180° - 130° = 50°
    FLHip.write(110);
    FLKnee.write(50);
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
    
    if(isRunning) {
        // Diagonal pair 1: FR-RL
        moveLegs(FRHip, FRKnee, RLHip, RLKnee, RRHip, RRKnee, FLHip, FLKnee);
        delay(100);
        
        // Diagonal pair 2: FL-RR
        moveLegs(RRHip, RRKnee, FLHip, FLKnee, FRHip, FRKnee, RLHip, RLKnee);
        delay(100);
    }
}


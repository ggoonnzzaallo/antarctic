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

bool isRunning = true;

void moveLegs(Servo& hipRight, Servo& kneeRight, Servo& hipLeft, Servo& kneeLeft) {
    // Phase 1: Lift legs
    // In this phase the leg moves all the way forward.
    hipRight.write(90);
    kneeRight.write(150);
    hipLeft.write(90);
    kneeLeft.write(30);    // Mirrored angle
    delay(300);
    
    // Phase 2: Plant forward
    // In this phase the leg goes all the way back to push forward
    hipRight.write(40);
    kneeRight.write(70);
    hipLeft.write(140);    // Mirrored angle
    kneeLeft.write(110);   // Mirrored angle
    delay(300);
    
    // Phase 3: Push back
    // I this phase the leg is in the air to create clearace
    hipRight.write(35);
    kneeRight.write(130);
    hipLeft.write(145);    // Mirrored angle
    kneeLeft.write(50);    // Mirrored angle
    delay(200);
    
    // Phase 4: Return to stance
    // In this phase the leg is in its standing position
    hipRight.write(70);
    kneeRight.write(130);
    hipLeft.write(110);
    kneeLeft.write(50);
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
        moveLegs(FRHip, FRKnee, RLHip, RLKnee);
        delay(100);
        
        // Diagonal pair 2: FL-RR
        moveLegs(RRHip, RRKnee, FLHip, FLKnee);
        delay(100);
    }
}



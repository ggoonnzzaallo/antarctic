#include <Servo.h>
#include <math.h>

// System Notes:
// For the Right side knee servos, higher values move the horn CCW
// For the Left side knee servos, higher values move the horn CCW



// Create servo objects
Servo FLKnee, FLHip, FLShoulder;
Servo FRKnee, FRHip, FRShoulder;
Servo RLKnee, RLHip, RLShoulder;
Servo RRKnee, RRHip, RRShoulder;

// System constants
const float Input_X = 100;  // in mm
const float Input_Y = 48.55;  // in mm
const float Input_Z = 250;  // in mm

const float Shoulder_to_foot = 48.55;  // in mm
const float Leg_upper = 130;  // in mm
const float Leg_lower = 143.82672;  // in mm

const float link_knee_L1 = 130;  // in mm
const float link_knee_L2 = 42.98015;  // in mm
const float link_knee_L3 = 130.2;  // in mm
const float link_knee_L4 = 37;  // in mm

const float link_servo_L1 = 35.2272;  // in mm
const float link_servo_L2 = 35;  // in mm
const float link_servo_L3 = 37.6;  // in mm
const float link_servo_L4 = 42.9;  // in mm
const float crank_angle = 102.5719779;  // in degrees

void calculateServoAngles(float input_x, float input_y, float input_z, int& shoulder_angle, int& hip_angle, int& knee_angle, bool isLeftSide) {
    // Y-Z Plane IK
    float var_D = sqrt(pow(input_y, 2) + pow(input_z, 2) - pow(Shoulder_to_foot, 2));
    float var_omega = (atan(input_y/input_z) + atan(var_D/Shoulder_to_foot));
    shoulder_angle = var_omega * 180/PI;

    // X-Z Plane IK
    float var_G = sqrt(pow(var_D, 2) + pow(input_x, 2));
    float var_Phi = acos((pow(var_G, 2) - pow(Leg_upper, 2) - pow(Leg_lower, 2))/(-2 * Leg_upper * Leg_lower));
    float var_Theta = atan(input_x/var_D) + asin((Leg_lower * sin(var_Phi))/var_G);

    Serial.print("var_G: ");
    Serial.println(var_G);
    Serial.print("var_Phi: ");
    Serial.println(var_Phi);
    Serial.print("var_Theta: ");
    Serial.println(var_Theta);
    
    // Different calculations for left and right sides
    hip_angle = isLeftSide ? (90 + var_Theta * 180/PI) : (90 - var_Theta * 180/PI);

    // 4-bar linkage, knee
    float link_knee_X = Leg_upper * sin(var_Theta);
    float link_knee_Y = Leg_upper * cos(var_Theta);
    float link_knee_Theta = atan(link_knee_Y/link_knee_X);
    float link_knee_Alpha = ((360-173.7924732)*PI/180) - var_Phi;
    float link_knee_L = sqrt(pow(link_knee_L1, 2) + pow(link_knee_L4, 2) - 2 * link_knee_L1 * link_knee_L4 * cos(link_knee_Alpha));
    float link_knee_Beta = asin((link_knee_L4 * sin(link_knee_Alpha))/link_knee_L);
    float link_knee_Lambda = acos((pow(link_knee_L3, 2) - pow(link_knee_L2, 2) - pow(link_knee_L, 2))/(-2 * link_knee_L2 * link_knee_L));
    float link_knee_Theta2 = link_knee_Beta + link_knee_Lambda - link_knee_Theta;

    Serial.print("link_knee_Theta: ");
    Serial.println(link_knee_Theta);
    Serial.print("link_knee_Alpha: ");
    Serial.println(link_knee_Alpha);
    Serial.print("link_knee_L: ");
    Serial.println(link_knee_L);
    Serial.print("link_knee_Beta: ");
    Serial.println(link_knee_Beta);
    Serial.print("link_knee_Lambda: ");
    Serial.println(link_knee_Lambda);
    Serial.print("link_knee_Theta2: ");
    Serial.println(link_knee_Theta2);

    // 4-bar linkage, servo horn
    float link_servo_X = 28.964;
    float link_servo_Y = 20.051;
    float link_servo_Theta = atan(link_servo_Y/link_servo_X);
    float link_servo_Alpha = link_servo_Theta + PI - (link_knee_Theta2 + (crank_angle * PI/180));
    float link_servo_L = sqrt(pow(link_servo_L4, 2) + pow(link_servo_L1, 2) - 2 * link_servo_L1 * link_servo_L4 * cos(link_servo_Alpha));
    float link_servo_Beta = acos((pow(link_servo_L, 2) + pow(link_servo_L1, 2) - pow(link_servo_L4, 2))/(2 * link_servo_L1 * link_servo_L));
    float link_servo_Lambda = acos((pow(link_servo_L2, 2) - pow(link_servo_L3, 2) + pow(link_servo_L, 2))/(2 * link_servo_L * link_servo_L2));
    float link_servo_Theta2 = link_servo_Theta + link_servo_Beta + link_servo_Lambda;
    
    Serial.print("link_servo_L: ");
    Serial.println(link_servo_L);
    Serial.print("link_servo_Alpha: ");
    Serial.println(link_servo_Alpha);
    Serial.print("link_servo_Beta: ");
    Serial.println(link_servo_Beta);
    Serial.print("link_servo_Theta: ");
    Serial.println(link_servo_Theta);
    Serial.print("link_servo_Lambda: ");
    Serial.println(link_servo_Lambda);
    
    // Different calculations for left and right sides
    knee_angle = isLeftSide ? (link_servo_Theta2 * 180/PI - 90) : (270 - link_servo_Theta2 * 180/PI);


    
}


void setup() {
    Serial.begin(9600);
    delay(1000);

    // Attach all servos
    RRShoulder.attach(A1);
    RRHip.attach(12);
    RRKnee.attach(11);
    
    RLShoulder.attach(10);
    RLHip.attach(9);
    RLKnee.attach(8);
    
    FRShoulder.attach(7);
    FRHip.attach(5);
    FRKnee.attach(6);
    
    FLShoulder.attach(4);
    FLHip.attach(2);
    FLKnee.attach(3);

    // Initialize shoulders to 90 degrees
    RRShoulder.write(90);
    RLShoulder.write(90);
    FRShoulder.write(90);
    FLShoulder.write(90);

    RRHip.write(70);
    RRKnee.write(130);
    FRHip.write(70);
    FRKnee.write(130);
    RLHip.write(110);
    RLKnee.write(50);
    FLHip.write(110);
    FLKnee.write(50);

    // Calculate and set initial positions
    int shoulder_angle, hip_angle, knee_angle;
    
    // Calculate for left legs
    calculateServoAngles(Input_X, Input_Y, Input_Z, shoulder_angle, hip_angle, knee_angle, true);
    // RLShoulder.write(shoulder_angle);
    // RLHip.write(hip_angle);
    // RLKnee.write(knee_angle);
    // FLShoulder.write(shoulder_angle);
    // FLHip.write(hip_angle);
    // FLKnee.write(knee_angle);

    // Calculate for right legs
    calculateServoAngles(Input_X, Input_Y, Input_Z, shoulder_angle, hip_angle, knee_angle, false);
    // RRShoulder.write(shoulder_angle);
    // RRHip.write(hip_angle);
    // RRKnee.write(knee_angle);
    // FRShoulder.write(shoulder_angle);
    // FRHip.write(hip_angle);
    // FRKnee.write(knee_angle);
}

// Define positions array [X, Y, Z]
const float positions[][4] = {
      {-60, 48.55, 170},  // Position 4
      {-60, 48.55,220}, // Position 3
      {0, 48.55, 220},    // Position 2
      {0, 48.55, 170}  // Position 1
};
const int NUM_POSITIONS = 4;
const int POSITION_DELAY = 100; // Delay between positions in ms

int currentPosition = 1;

void loop() {
    int shoulder_angle, hip_angle, knee_angle;
    
    Serial.println("\n--------------------------------------------------");
    Serial.print("Moving to position ");
    Serial.print(currentPosition + 1);
    Serial.print(" [X: ");
    Serial.print(positions[currentPosition][0]);
    Serial.print(", Y: ");
    Serial.print(positions[currentPosition][1]);
    Serial.print(", Z: ");
    Serial.print(positions[currentPosition][2]);
    Serial.println("]");
    
    calculateServoAngles(
        positions[currentPosition][0],  // X
        positions[currentPosition][1],  // Y
        positions[currentPosition][2],  // Z
        shoulder_angle, 
        hip_angle, 
        knee_angle, 
        true
    );

    Serial.println("Commanding RR servos to:");
    Serial.print("RRShoulder: ");
    Serial.print(shoulder_angle);
    Serial.print("° | RRHip: ");
    Serial.print(hip_angle);
    Serial.print("° | RRKnee: ");
    Serial.println(knee_angle);
    Serial.println("--------------------------------------------------");

    FLShoulder.write(shoulder_angle);
    FLHip.write(hip_angle);a
    FLKnee.write(knee_angle);

    currentPosition = (currentPosition + 1) % NUM_POSITIONS;
    delay(POSITION_DELAY);
}



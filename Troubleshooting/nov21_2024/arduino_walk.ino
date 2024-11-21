#include <Servo.h>
#include <math.h>

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

enum LegState {
    GROUNDED = 0,
    MOVING = 1
};

struct ServoTrim {
    int shoulder;
    int hip; 
    int knee;
};

struct Leg {
    Servo shoulder, hip, knee;
    ServoTrim trim;
    int currentPosition;
    LegState state;
    bool isLeft;
    bool isFront;
};

ServoTrim FL_trim = {0, 0, 0};  
ServoTrim FR_trim = {0, 10, -10};
ServoTrim RL_trim = {0, 0, 0};
ServoTrim RR_trim = {-7, 0, 10};

Leg FL = {FLShoulder, FLHip, FLKnee, FL_trim, 0, GROUNDED, true, true};
Leg RR = {RRShoulder, RRHip, RRKnee, RR_trim, 0, GROUNDED, false, false};
Leg FR = {FRShoulder, FRHip, FRKnee, FR_trim, 0, GROUNDED, false, true};
Leg RL = {RLShoulder, RLHip, RLKnee, RL_trim, 0, GROUNDED, true, false};

Leg* legs[4] = {&FL, &FR, &RL, &RR}; // Front left -> Front right -> Rear left -> Rear right

const float legPositions[4][3] = {
    {-40, 70, 200},  // FL custom position
    {-40, 70, 200},  // FR custom position
    {60, 70, 200},   // RL custom position
    {60, 70, 200}    // RR custom position
};

const float frontLegSteps[3][3] = {
    {20, 70, 200},  // Position 1, Ground position
    {-10, 70, 180}, // Position 2, Lift position
    {-40, 70, 200}    // Position 3, Forward position
};

const float rearLegSteps[3][3] = {
    {120, 70, 200},   // Position 1, Ground position
    {90, 70, 180},  // Position 2, Lift position
    {60, 70, 200}   // Position 3, Forward position
};


void calculateServoAngles(float input_x, float input_y, float input_z, int& shoulder_angle, int& hip_angle, int& knee_angle, bool isLeftSide, bool isFront) {
    // Y-Z Plane IK
    float var_D = sqrt(pow(input_y, 2) + pow(input_z, 2) - pow(Shoulder_to_foot, 2));
    float var_omega = (atan(input_y/input_z) + atan(var_D/Shoulder_to_foot));
    
    // RR is our reference (positive moves outward)
    if (isFront && isLeftSide)      shoulder_angle = var_omega * 180/PI;      // FL outward
    else if (isFront && !isLeftSide) shoulder_angle = 180 -var_omega * 180/PI;    // FR inward
    else if (!isFront && isLeftSide) shoulder_angle = 180 -var_omega * 180/PI;    // RL inward
    else                            shoulder_angle = var_omega * 180/PI;      // RR outward

    // X-Z Plane IK
    float var_G = sqrt(pow(var_D, 2) + pow(input_x, 2));
    float var_Phi = acos((pow(var_G, 2) - pow(Leg_upper, 2) - pow(Leg_lower, 2))/(-2 * Leg_upper * Leg_lower));
    float var_Theta = atan(input_x/var_D) + asin((Leg_lower * sin(var_Phi))/var_G);
    
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

    // 4-bar linkage, servo horn
    float link_servo_X = 28.964;
    float link_servo_Y = 20.051;
    float link_servo_Theta = atan(link_servo_Y/link_servo_X);
    float link_servo_Alpha = link_servo_Theta + PI - (link_knee_Theta2 + (crank_angle * PI/180));
    float link_servo_L = sqrt(pow(link_servo_L4, 2) + pow(link_servo_L1, 2) - 2 * link_servo_L1 * link_servo_L4 * cos(link_servo_Alpha));
    float link_servo_Beta = acos((pow(link_servo_L, 2) + pow(link_servo_L1, 2) - pow(link_servo_L4, 2))/(2 * link_servo_L1 * link_servo_L));
    float link_servo_Lambda = acos((pow(link_servo_L2, 2) - pow(link_servo_L3, 2) + pow(link_servo_L, 2))/(2 * link_servo_L * link_servo_L2));
    float link_servo_Theta2 = link_servo_Theta + link_servo_Beta + link_servo_Lambda;
    
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
    RRShoulder.write(100);
    RLShoulder.write(100);
    FRShoulder.write(89);
    FLShoulder.write(80);

    RRHip.write(70);
    RRKnee.write(130);
    FRHip.write(70);
    FRKnee.write(130);
    RLHip.write(110);
    RLKnee.write(50);
    FLHip.write(110);
    FLKnee.write(50);

    // Initialize all legs to starting positions
    for(int i = 0; i < 4; i++) {
        int shoulder_angle, hip_angle, knee_angle;
        calculateServoAngles(
            legPositions[i][0],
            legPositions[i][1],
            legPositions[i][2],
            shoulder_angle, 
            hip_angle, 
            knee_angle, 
            legs[i]->isLeft,
            legs[i]->isFront
        );
        legs[i]->shoulder.write(shoulder_angle);
        legs[i]->hip.write(hip_angle);
        legs[i]->knee.write(knee_angle);
    }
}

void loop() {
    static int diag1Step = 0;  // FL+RR pair
    static int diag2Step = 2;  // FR+RL pair, starts at position 2
    
    Serial.print("Diag1(FL+RR): "); Serial.print(diag1Step);
    Serial.print(" | Diag2(FR+RL): "); Serial.println(diag2Step);
    
    // Move diagonal pair 1 (FL + RR)
    moveLeg(*legs[0], frontLegSteps[diag1Step]);  // FL
    moveLeg(*legs[3], rearLegSteps[diag1Step]);   // RR
    
    // Move diagonal pair 2 (FR + RL)
    moveLeg(*legs[1], frontLegSteps[diag2Step]);  // FR
    moveLeg(*legs[2], rearLegSteps[diag2Step]);   // RL
    
    delay(1000);
    
    // Explicit step transitions
    if (diag1Step == 2) diag1Step = 0;
    else diag1Step++;
    
    if (diag2Step == 2) diag2Step = 0;
    else diag2Step++;
}

void moveLeg(Leg& leg, float* position) {
    int shoulder_angle, hip_angle, knee_angle;
    calculateServoAngles(position[0], position[1], position[2], 
                        shoulder_angle, hip_angle, knee_angle, leg.isLeft, leg.isFront);
    
    leg.shoulder.write(shoulder_angle + leg.trim.shoulder);
    leg.hip.write(hip_angle + leg.trim.hip);
    leg.knee.write(knee_angle + leg.trim.knee);
    leg.currentPosition = position[0]; // Track X position or whatever you prefer
}


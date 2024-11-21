
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

enum RobotState {
    WALKING,
    SWIMMING,
    DANCING
};

struct Leg {
    Servo shoulder, hip, knee;
    int currentPosition;
    LegState state;
    bool isLeft;
};

Leg FL = {FLShoulder, FLHip, FLKnee, 0, GROUNDED, true};
Leg RR = {RRShoulder, RRHip, RRKnee, 0, GROUNDED, false};
Leg FR = {FRShoulder, FRHip, FRKnee, 0, GROUNDED, false};
Leg RL = {RLShoulder, RLHip, RLKnee, 0, GROUNDED, true};

Leg* legs[4] = {&FL, &RR, &FR, &RL};

const float positions[][3] = {
    {0, 48.55, 220},     // Position 1, back and grounded
    {0, 48.55, 170},     // Position 2, back and lifted
    {-60, 48.55, 170},    // Position 3, front and lifted
    {-60, 48.55, 220}   // Position 4, front and grounded

};

void calculateServoAngles(float input_x, float input_y, float input_z, int& shoulder_angle, int& hip_angle, int& knee_angle, bool isLeftSide) {
    // Y-Z Plane IK
    float var_D = sqrt(pow(input_y, 2) + pow(input_z, 2) - pow(Shoulder_to_foot, 2));
    float var_omega = (atan(input_y/input_z) + atan(var_D/Shoulder_to_foot));
    shoulder_angle = var_omega * 180/PI;

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

void walkingRoutine() {
    static int activeleg = 0;  // 0=FL, 1=RR, 2=FR, 3=RL

    Leg* currentLeg = legs[activeleg];
    
    if (currentLeg->currentPosition == 3) {
        // When leg is in air (pos 3), start moving next leg
        Leg* nextLeg = legs[(activeleg + 1) % 4];
        if (nextLeg->currentPosition < 2) {
            moveLeg(*nextLeg, positions[nextLeg->currentPosition]);
            nextLeg->currentPosition++;
            return;
        }
    }
    
    if (currentLeg->currentPosition == 4) {
        moveLeg(*currentLeg, positions[0]);  // Ground contact movement (4->1)
        currentLeg->currentPosition = 0;
        activeleg = (activeleg + 1) % 4;
    } else if (currentLeg->currentPosition < 4) {
        moveLeg(*currentLeg, positions[currentLeg->currentPosition]);
        currentLeg->currentPosition++;
    }
    
    delay(500);
}

void swimmingRoutine() {
    static int activeleg = 0;
    static unsigned long lastTime = 0;
    static float phase = 0;
    const float frequency = 0.05;  // Lower = smoother
    
    Leg* currentLeg = legs[activeleg];
    
    // Smooth sine wave interpolation
    phase += frequency;
    if(phase > TWO_PI) phase -= TWO_PI;
    
    float shoulderAngle1 = 90 + 22.5 * (sin(phase) + 1);  // FL and RR: 90 to 135
    float shoulderAngle2 = 90 - 22.5 * (sin(phase) + 1);  // FR and RL: 90 to 45
    
    FL.shoulder.write(shoulderAngle1);
    RR.shoulder.write(shoulderAngle1);
    FR.shoulder.write(shoulderAngle2);
    RL.shoulder.write(shoulderAngle2);
    
    // Rest of walking logic remains unchanged
    if (currentLeg->currentPosition == 3) {
        Leg* nextLeg = legs[(activeleg + 1) % 4];
        if (nextLeg->currentPosition < 2) {
            moveLeg(*nextLeg, positions[nextLeg->currentPosition]);
            nextLeg->currentPosition++;
            return;
        }
    }
    
    if (currentLeg->currentPosition == 4) {
        moveLeg(*currentLeg, positions[0]);
        currentLeg->currentPosition = 0;
        activeleg = (activeleg + 1) % 4;
    } else if (currentLeg->currentPosition < 4) {
        moveLeg(*currentLeg, positions[currentLeg->currentPosition]);
        currentLeg->currentPosition++;
    }
    
    delay(20);  // Shorter delay for smoother motion
}

void dancingRoutine() {
    static int activeleg = 0;  // 0=FL, 1=RR, 2=FR, 3=RL

    Leg* currentLeg = legs[activeleg];
    
    if (currentLeg->currentPosition == 3) {
        // When leg is in air (pos 3), start moving next leg
        Leg* nextLeg = legs[(activeleg + 1) % 4];
        if (nextLeg->currentPosition < 2) {
            moveLeg(*nextLeg, positions[nextLeg->currentPosition]);
            nextLeg->currentPosition++;
            return;
        }
    }
    
    if (currentLeg->currentPosition == 4) {
        moveLeg(*currentLeg, positions[0]);  // Ground contact movement (4->1)
        currentLeg->currentPosition = 0;
        activeleg = (activeleg + 1) % 4;
    } else if (currentLeg->currentPosition < 4) {
        moveLeg(*currentLeg, positions[currentLeg->currentPosition]);
        currentLeg->currentPosition++;
    }
    
    delay(500);
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
            positions[0][0],
            positions[0][1],
            positions[0][2],
            shoulder_angle, 
            hip_angle, 
            knee_angle, 
            legs[i]->isLeft
        );
        legs[i]->shoulder.write(shoulder_angle);
        legs[i]->hip.write(hip_angle);
        legs[i]->knee.write(knee_angle);
    }
}

void loop() {
    static RobotState currentState = SWIMMING;
    static unsigned long stateStartTime = millis();
    
    switch(currentState) {
        // case WALKING:
        //     walkingRoutine();
        //     if(millis() - stateStartTime > 5000) {
        //         currentState = SWIMMING;
        //         stateStartTime = millis();
        //     }
        //     break;
            
        case SWIMMING:
            swimmingRoutine();
            if(millis() - stateStartTime > 3000) {
                currentState = SWIMMING;
                stateStartTime = millis();
            }
            break;
            
        // case DANCING:
        //     dancingRoutine();
        //     if(millis() - stateStartTime > 4000) {
        //         currentState = WALKING;
        //         stateStartTime = millis();
        //     }
        //     break;
    }
}

void moveLeg(Leg& leg, float* position) {
    int shoulder_angle, hip_angle, knee_angle;
    calculateServoAngles(position[0], position[1], position[2], 
                        shoulder_angle, hip_angle, knee_angle, leg.isLeft);
    
    leg.shoulder.write(shoulder_angle);
    leg.hip.write(hip_angle);
    leg.knee.write(knee_angle);
}


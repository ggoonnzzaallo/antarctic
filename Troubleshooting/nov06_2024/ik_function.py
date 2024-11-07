import math

def calculate_leg_angles(Input_X: float, Input_Y: float, Input_Z: float, side: str) -> tuple[float, float, float]:
    """
    ###INSTRUCTIONS###
    We want to go from X, Y, Z coordinates and solve for the servo angels
    We will begin with the rear-right leg
    X-axis is Front-Back
    Y-axis is Left-Right
    Z-axis is Top-Bottom
    """
    if side not in ['Left', 'Right']:
        raise ValueError("Side must be 'Left' or 'Right'")

    #Defining our system constants:
    Shoulder_to_foot = 48.55 #in mm
    Leg_upper = 130 #in mm
    Leg_lower = 143.82672 #in mm

    link_knee_L1 = 130 #in mm
    link_knee_L2 = 42.98015 #in mm
    link_knee_L3 = 130.2 #in mm
    link_knee_L4 = 37 #in mm

    link_servo_L1 = 35.2272 #in mm
    link_servo_L2 = 35 #in mm
    link_servo_L3 = 37.6 #in mm
    link_servo_L4 = 42.9 #in mm
    crank_angle = 102.5719779 #in degrees

    #Y-Z Plane IK
    var_D = math.sqrt((Input_Y**2 + Input_Z**2)-Shoulder_to_foot**2)
    var_omega = (math.atan(Input_Y/Input_Z) + (math.atan(var_D/Shoulder_to_foot)))
    

    #X-Z Plane IK
    var_G = math.sqrt(var_D**2 + Input_X**2)
    var_Phi = math.acos((var_G**2-(Leg_upper**2)-(Leg_lower**2))/(-2*Leg_upper*Leg_lower))
    var_Theta = math.atan(Input_X/var_D) + math.asin((Leg_lower*math.sin(var_Phi))/var_G)

    #4-bar linkage, knee
    link_knee_X = Leg_upper*math.sin(var_Theta)
    link_knee_Y = Leg_upper*math.cos(var_Theta)
    link_knee_Theta = math.atan(link_knee_Y/link_knee_X)
    link_knee_Alpha = math.pi - var_Phi
    link_knee_L = math.sqrt((link_knee_L1**2)+(link_knee_L4**2)-(2*link_knee_L1*link_knee_L4*math.cos(link_knee_Alpha)))
    link_knee_Beta = math.asin((link_knee_L4*math.sin(link_knee_Alpha))/link_knee_L)
    link_knee_Lambda = math.acos(((link_knee_L3**2)-(link_knee_L2**2)-(link_knee_L**2))/(-2*link_knee_L2*link_knee_L))
    link_knee_Theta2 = link_knee_Beta+link_knee_Lambda-link_knee_Theta

    #4-bar linkage, servo horn
    link_servo_X = 28.964
    link_servo_Y = 20.051
    link_servo_Theta = math.atan(link_servo_Y/link_servo_X)
    link_servo_Alpha = link_servo_Theta+(math.pi)-(link_knee_Theta2+((crank_angle*math.pi)/(180)))
    link_servo_L = math.sqrt((link_servo_L4**2)+(link_servo_L1**2)-(2*link_servo_L1*link_servo_L4*math.cos(link_servo_Alpha)))
    link_servo_Beta = math.acos(((link_servo_L**2)+(link_servo_L1**2)-(link_servo_L4**2))/(2*link_servo_L1*link_servo_L))
    link_servo_Lambda = math.acos((link_servo_L2**2-link_servo_L3**2+link_servo_L**2)/(2*link_servo_L*link_servo_L2))
    link_servo_Theta2 = link_servo_Theta+link_servo_Beta+link_servo_Lambda

    # Calculate final servo angles based on side
    if side == 'Left':
        servo_hip_deg = 90 + (var_Theta*180/math.pi)
        servo_knee_deg = 270 - (link_servo_Theta2*180/math.pi)
        servo_shoulder_deg = 180-var_omega*180/math.pi
    else:  # Right side
        servo_hip_deg = 90 - (var_Theta*180/math.pi)
        servo_knee_deg = (link_servo_Theta2*180/math.pi) - 90
        servo_shoulder_deg = var_omega*180/math.pi

    return servo_shoulder_deg, servo_hip_deg, servo_knee_deg

# Example usage
if __name__ == "__main__":
    angles = calculate_leg_angles(55.686, 48.55, 122.852, 'Right') #X, Y, Z
    print(f"Servo angles - Shoulder: {int(angles[0])}°, Hip: {int(angles[1])}°, Knee: {int(angles[2])}°")



# Moving to position 1 [X: -30.00, Y: 48.55, Z: 180.00]
# Commanding RR servos to:
# RRShoulder: 89° | RRHip: 47° | RRKnee: 54
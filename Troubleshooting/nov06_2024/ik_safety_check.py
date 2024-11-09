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
    link_knee_Alpha = ((360-173.7924732)*math.pi/180) - var_Phi
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
        servo_knee_deg = link_servo_Theta2*180/math.pi -90
        servo_shoulder_deg = 180-var_omega*180/math.pi
    else:  # Right side
        servo_hip_deg = 90 - (var_Theta*180/math.pi)
        servo_knee_deg = 270 - (link_servo_Theta2*180/math.pi)
        servo_shoulder_deg = var_omega*180/math.pi




    return servo_shoulder_deg, servo_hip_deg, servo_knee_deg

def check_servo_angles(hip_angle: float, knee_angle: float, shoulder_angle: float) -> tuple[bool, str]:
    # Check basic angle ranges (0-180)
    if not (0 <= hip_angle <= 180 and 0 <= knee_angle <= 180 and 0 <= shoulder_angle <= 180):
        return False, "Angles must be between 0-180°"
        
    # Define safe ranges (hip angle: [min knee, max knee])
    safe_ranges = {
        0: [40, 140], 5: [40, 155], 10: [40, 150], 15: [20, 165], 
        20: [20, 165], 25: [20, 165], 30: [40, 165], 35: [60, 165],
        40: [60, 165], 45: [70, 165], 50: [80, 160], 55: [80, 160],
        60: [90, 165], 65: [90, 165], 70: [100, 160], 75: [110, 160],
        80: [110, 160], 85: [120, 160], 90: [125, 160], 95: [135, 160],
        100: [140, 160], 105: [145, 160], 110: [145, 160], 115: [160, 165],
        120: [165, 165], 125: [165, 165]
    }
    
    hip_rounded = round(hip_angle / 5) * 5
    hip_rounded = min(max(0, hip_rounded), 125)
    
    min_knee, max_knee = safe_ranges[hip_rounded]
    if not (min_knee <= knee_angle <= max_knee):
        return False, "Knee angle unsafe for given hip position"
        
    return True, "Safe"

# Example usage
if __name__ == "__main__":
    # Define square parameters
    start_point = [-60, 48.55, 170]  # Starting corner [X, Y, Z]
    square_base = 60  # X-axis length
    square_height = 50  # Z-axis length
    side = 'Right'
    
    square_points = [
        start_point,
        [start_point[0] + square_base, start_point[1], start_point[2]],
        [start_point[0] + square_base, start_point[1], start_point[2] + square_height],
        [start_point[0], start_point[1], start_point[2] + square_height]
    ]
    
    for point in square_points:
        angles = calculate_leg_angles(*point, side)
        is_safe, message = check_servo_angles(angles[1], angles[2], angles[0])
        print(f"Coords {point} -> Angles [S:{int(angles[0])}°, H:{int(angles[1])}°, K:{int(angles[2])}°] {'✅' if is_safe else f'❌ {message}'}")



#For the Right side knee servos, higher values move the horn CCW




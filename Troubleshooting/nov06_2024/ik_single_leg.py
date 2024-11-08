import math

###INSTRUCTIONS###
#We want to go from X, Y, Z coordinates and solve for the servo angels
#We will begin with the rear-right leg
#X-axis is Front-Back
#Y-axis is Left-Right
#Z-axis is Top-Bottom

#Defining our inputs:
Input_X = 50 #in mm
Input_Y = 48.55 #in mm
Input_Z = 120 #in mm

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
print("Variable D is: (mm)", var_D)
var_omega = (math.atan(Input_Y/Input_Z) + (math.atan(var_D/Shoulder_to_foot)))
print("Variable Omega is: (rad)", var_omega)
print("Variable Omega is: (deg)", var_omega*180/math.pi) 
servo_shoulder_deg = var_omega*180/math.pi #<- ATTENTION: This is the input for the shoulder servo

#X-Z Plane IK
var_G = math.sqrt(var_D**2 + Input_X**2)
print("Variable G is: (mm)", var_G)
var_Phi = math.acos((var_G**2-(Leg_upper**2)-(Leg_lower**2))/(-2*Leg_upper*Leg_lower))
print("Variable Phi is: (rad)", var_Phi)
print("Variable Phi is: (deg)", var_Phi*180/math.pi)
var_Theta = math.atan(Input_X/var_D) + math.asin((Leg_lower*math.sin(var_Phi))/var_G)
print("Variable Theta is: (rad)", var_Theta)
print("Variable Theta is: (deg)", var_Theta*180/math.pi) 
servo_hip_deg = 90 - var_Theta*180/math.pi #<- ATTENTION: This is the input for the hip servo, right
#servo_hip_deg = 90 + var_Theta*180/math.pi #<- ATTENTION: This is the input for the hip servo, left

#4-bar linkage, knee
link_knee_X = Leg_upper*math.sin(var_Theta)
link_knee_Y = Leg_upper*math.cos(var_Theta)
print("Link knee X is: (mm)", link_knee_X)
print("Link knee Y is: (mm)", link_knee_Y)
link_knee_Theta = math.atan(link_knee_Y/link_knee_X)
print("Link knee Theta is: (rad)", link_knee_Theta)
print("Link knee Theta is: (deg)", link_knee_Theta*180/math.pi)
link_knee_Alpha = ((360-173.7924732)*math.pi/180) - var_Phi
print("Link knee Alpha is: (rad)", link_knee_Alpha)
print("Link knee Alpha is: (deg)", link_knee_Alpha*180/math.pi)
link_knee_L = math.sqrt((link_knee_L1**2)+(link_knee_L4**2)-(2*link_knee_L1*link_knee_L4*math.cos(link_knee_Alpha)))
print("Link knee L is: (mm)", link_knee_L)
link_knee_Beta = math.asin((link_knee_L4*math.sin(link_knee_Alpha))/link_knee_L)
print("Link knee Beta is: (rad)", link_knee_Beta)
link_knee_Lambda =math.acos(((link_knee_L3**2)-(link_knee_L2**2)-(link_knee_L**2))/(-2*link_knee_L2*link_knee_L))
print("Link knee Lambda is: (rad)", link_knee_Lambda)
link_knee_Theta2 =link_knee_Beta+link_knee_Lambda-link_knee_Theta
print("Link knee Theta2 is: (rad)", link_knee_Theta2)
print("Link knee Theta2 is: (deg)", link_knee_Theta2*180/math.pi)

#4-bar linkage, servo horn
link_servo_X = 28.964
link_servo_Y = 20.051
link_servo_Theta = math.atan(link_servo_Y/link_servo_X)
print("Link servo Theta is: (rad)", link_servo_Theta)
print("Link servo Theta is: (deg)", link_servo_Theta*180/math.pi)
link_servo_Alpha = link_servo_Theta+(math.pi)-(link_knee_Theta2+((crank_angle*math.pi)/(180)))
print("Link servo Alpha is: (rad)", link_servo_Alpha)
link_servo_L = math.sqrt((link_servo_L4**2)+(link_servo_L1**2)-(2*link_servo_L1*link_servo_L4*math.cos(link_servo_Alpha)))
print("Link servo L is: (mm)", link_servo_L)
link_servo_Beta = math.acos(((link_servo_L**2)+(link_servo_L1**2)-(link_servo_L4**2))/(2*link_servo_L1*link_servo_L))
print("Link servo Beta is: (rad)", link_servo_Beta)
print("Link servo Beta is: (deg)", link_servo_Beta*180/math.pi)
link_servo_Lambda = math.acos((link_servo_L2**2-link_servo_L3**2+link_servo_L**2)/(2*link_servo_L*link_servo_L2))
print("Link servo Lambda is: (rad)", link_servo_Lambda)
link_servo_Theta2 = link_servo_Theta+link_servo_Beta+link_servo_Lambda
print("Link servo Theta2 is: (rad)", link_servo_Theta2)
print("Link servo Theta2 is: (deg)", link_servo_Theta2*180/math.pi) 
servo_knee_deg = 270 - (link_servo_Theta2*180/math.pi) #<- ATTENTION: This is the input for the knee servo, right
#servo_knee_deg = link_servo_Theta2*180/math.pi -90 #<- ATTENTION: This is the input for the knee servo, left

print(f"Servo angles - Shoulder: {int(servo_shoulder_deg)}°, Hip: {int(servo_hip_deg)}°, Knee: {int(servo_knee_deg)}°")



# # ---------------------------------------------------
# Moving to position 2 [X: 50.00, Y: 48.55, Z: 120.00]
# var_D: 120.00
# var_omega: 1.57
# var_G: 130.00
# var_Phi: 0.98
# var_Theta: 1.57
# link_knee_Theta: 0.00
# link_knee_Alpha: 2.27
# link_knee_L: 156.29
# link_knee_Beta: 0.18
# link_knee_Lambda: 0.80
# link_knee_Theta2: 0.98
# link_servo_L: 37.17
# link_servo_Alpha: 0.97
# link_servo_Beta: 1.27
# link_servo_Theta: 0.61
# link_servo_Lambda: 1.09


#Left knee 80 degrees, moving CCW
#Right knee is 99 degrees, moving CCW
import pybullet as p
import pybullet_data
import time

# Initialize PyBullet in GUI mode without UI elements
physicsClient = p.connect(p.GUI, options="--width=1920 --height=1080 --background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0")

# Configure visualization
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Disable UI overlay
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)  # Disable keyboard shortcuts
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)  # Enable mouse picking

# Set up search path for URDFs
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.81)

# Load plane to act as ground
planeId = p.loadURDF("plane.urdf")

# Load your robot URDF
# Replace with your URDF path
#robotId = p.loadURDF("my-robot/robot.urdf")
robotId = p.loadURDF("robot/urdf_robot.urdf")

# Get the position of the robot
robot_pos, robot_orient = p.getBasePositionAndOrientation(robotId)

# Set camera to focus on the robot
# Parameters: target position, distance from target, yaw, pitch, up vector
p.resetDebugVisualizerCamera(
    cameraDistance=2.0,  # Distance from the target (adjust as needed)
    cameraYaw=45.0,     # Camera rotation around the z-axis in degrees
    cameraPitch=-30.0,  # Camera pitch in degrees
    cameraTargetPosition=robot_pos  # Point the camera at the robot's position
)

# Modified simulation loop with mouse force application
prev_mouse_pos = None
for i in range(100000):
    # Get mouse events
    mouse_events = p.getMouseEvents()
    for e in mouse_events:
        if e[0] == 1:  # Left mouse button event (1 = left button)
            if e[1] == 3:  # Mouse button down (3 = button down)
                # Get hit position and body when mouse is clicked
                mouse_pos = e[3]  # Mouse position is in e[3]
                ray_start = mouse_pos
                ray_end = [mouse_pos[0], mouse_pos[1], -1000.0]  # Cast ray downward
                ray_info = p.rayTest(ray_start, ray_end)[0]
                if ray_info[0] == robotId:  # If we hit the robot
                    prev_mouse_pos = mouse_pos
            elif e[1] == 4:  # Mouse button up (4 = button up)
                prev_mouse_pos = None
    
    # Apply force if dragging
    if prev_mouse_pos is not None:
        # Get current mouse position
        mouse_pos = p.getMouseEvents()[-1][3]
        if mouse_pos != prev_mouse_pos:
            # Calculate force based on mouse movement
            force_scale = 50.0  # Adjust this to change force magnitude
            force = [(mouse_pos[0] - prev_mouse_pos[0]) * force_scale,
                    (mouse_pos[1] - prev_mouse_pos[1]) * force_scale,
                    0]
            # Apply the force at the clicked point
            p.applyExternalForce(robotId, -1, force, mouse_pos, p.WORLD_FRAME)
            prev_mouse_pos = mouse_pos

    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect when done
p.disconnect()
'''
This script is not part of training or evaluating, it's just to test the model and make sure everything loads correctly. Can ignore it.
'''

import mujoco
import mujoco.viewer
import numpy as np
import time

# Load the model - use the full path to your XML file
model = mujoco.MjModel.from_xml_path('scene.xml')  # or 'scene.xml'
data = mujoco.MjData(model)

# Reset the simulation state before starting
mujoco.mj_resetData(model, data)

# Set initial positions for the root (free joint)
data.qpos[0:3] = [0, 0, 0.1]  # x, y, z position (z = 0.1 meters = 10 cm)
data.qpos[3:7] = [1, 0, 0, 0]  # quaternion orientation (w, x, y, z)

# Set initial positions for other joints
joint_names = [
    "dof_shoulder_front_left",
    "dof_hip_front_left",
    "dof_knee_front_left",
    "dof_shoulder_front_right",
    "dof_hip_front_right",
    "dof_knee_front_right",
    "dof_shoulder_rear_right",
    "dof_hip_rear_right",
    "dof_knee_rear_right",
    "dof_shoulder_rear_left",
    "dof_hip_rear_left",
    "dof_knee_rear_left"
]

# Set desired positions for each joint
joint_positions = [1.57, 0.0, 0.0, 1.57, 0.0, 3.14, -1.57, 1.57, 3.14, 1.57, 0.0, 0.0]

# Set each joint position
for name, pos in zip(joint_names, joint_positions):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id == -1:
        print(f"Warning: Joint '{name}' not found in model")
        continue
    # Get the correct qpos address for this joint
    joint_addr = model.jnt_qposadr[joint_id]
    data.qpos[joint_addr] = pos

test_joint = "dof_shoulder_front_left"
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, test_joint)
actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, test_joint)
print(f"\nActuator ID for {test_joint}: {actuator_id}")

# Print the limits
joint_range = model.jnt_range[joint_id]
print(f"Joint limits: [{joint_range[0]:.3f}, {joint_range[1]:.3f}] radians")
print(f"Actuator control range: [{model.actuator_ctrlrange[actuator_id][0]:.1f}, {model.actuator_ctrlrange[actuator_id][1]:.1f}]")

print(f"Initial joint position: {data.qpos[model.jnt_qposadr[joint_id]]}")
print(f"Joint damping: {model.dof_damping[model.jnt_dofadr[joint_id]]}")
print(f"Joint frictionloss: {model.dof_frictionloss[model.jnt_dofadr[joint_id]]}")

# Initialize control to current position to prevent sudden jumps
# initial_pos = data.qpos[model.jnt_qposadr[joint_id]]
# data.ctrl[actuator_id] = initial_pos

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    
    while viewer.is_running():
        # Get the control signal limits (now represents torque limits)
        ctrl_range = model.actuator_ctrlrange[actuator_id]
        print(f"\nTorque range: [{ctrl_range[0]:.3f}, {ctrl_range[1]:.3f}]")
        
        # Apply torque command (example: sinusoidal torque)
        time_now = time.time() - start_time
        desired_torque = 50 * np.sin(time_now)  # Oscillate between -50 and 50 N⋅m
        data.ctrl[actuator_id] = desired_torque
        
        # Step the simulation
        mujoco.mj_step(model, data)
        
        # Get actual state
        actual_position = data.qpos[model.jnt_qposadr[joint_id]]
        actual_velocity = data.qvel[model.jnt_dofadr[joint_id]]
        applied_torque = data.actuator_force[actuator_id]
        
        print(f"Applied torque: {desired_torque:.3f}")
        print(f"Actual pos: {actual_position:.3f}")
        print(f"Velocity: {actual_velocity:.3f}")
        print(f"Measured torque: {applied_torque:.3f}")
        
        viewer.sync()
        time.sleep(0.01)

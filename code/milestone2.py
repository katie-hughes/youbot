"""
GENERATE CSV WITH: 
python3 milestone2.py
"""

import modern_robotics as mr
import numpy as np


# MILESTONE 1

def NextState(current_config, controls, dt, joint_threshold):
    """
    Steps the robot forward
    
    Args
        current_config: 12 vector representing current configuraiton of the robot.
            3 variables for the chassis configuration, 5 variables for the arm configuration,
            and 4 variables for the wheel angles.
        controls: 9 vector indicating wheel speeds u (4 variables)
            and arm joint speeds thetadot (5 variables)
        dt: timestep (sec)
        joint_threshold: positive real value indicating the maximum angular speed
            of arm joints and wheels
    Returns:
        new_config: configuration after timestep dt based on euler step
    """
    # Convert to np array to make operations easier
    current_config = np.array(current_config)
    controls = np.array(controls)
    # Extract out specific configs
    current_chassis_config = current_config[:3]
    current_arm_config = current_config[3:8]
    current_wheel_config = current_config[8:]
    # Extract out speeds
    wheel_speeds = controls[:4]
    arm_speeds = controls[4:]
    # Compute new configs
    new_arm_config = current_arm_config + arm_speeds*dt
    new_wheel_config = current_wheel_config + wheel_speeds*dt
    # Calculate based on odometry
    new_chassis_config = current_chassis_config
    # Combine back together
    return np.concatenate([new_chassis_config, new_arm_config, new_wheel_config])


dt = 0.01
for i in range(0,100):



# MILESTONE 2

def csv_format(traj, gripper):
    """Turns the trajectory returned by the mr functions into the scene8 format."""
    res = []
    for i in traj:
        newline = [i[0][0], i[0][1], i[0][2],
                   i[1][0], i[1][1], i[1][2],
                   i[2][0], i[2][1], i[2][2],
                   i[0][3], i[1][3], i[2][3],
                   gripper]
        res.append(newline)
    return res

def get_T_N(T1, T2, speed, freq):
    """Calculates the time and number of points for the trajectory based on distance to travel."""
    p1 = T1[0:3,3]
    p2 = T2[0:3,3]
    # Linear distance between 2 transformation matrices
    dist = np.linalg.norm(p2-p1)
    T = round(dist/speed, 2) 
    N = int(T*freq)
    return T, N


def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k):
    """
    Generates trajectory

    Args:
        Tse_init: Initial configuration of the end effector in the reference trajectory
        Tsc_init: Cube's initial configuration
        Tsc_final: Cube's desired final configuration
        Tce_grasp: End effector's configuration relative to cube when grasping cube 
        Tce_standoff: End effector's standoff configuration above the cube, relative to the cube
        k: Number of trajectory configurations per 0.01 seconds. Must be 1 or greater. 
    Returns: 
        traj : A representation of the N configurations of the end-effector along the entire
               concatenated 8-segment trajectory. Each point represents a transformation Tse
               of the end effector frame e relative to s at an instant in time, plus the gripper 
               state (0, 1).
    """
    # Use 5th order polynomial
    method = 5
    # Gripper state (starts open)
    gripper = 0
    # Time to close the gripper
    gripper_close = 0.625
    # With current implemenation going up/down to standoff is super fast. Slow it down a bit
    standoff_scale = 5
    # Speed of arm in m/s
    speed = 0.5 
    # Sampling frequency in points/s
    freq = 100*k 

    # 1. Go from Tse_init to Tse_standoff
    print("\nSTEP 1: Move to first standoff")
    Tse_standoff = Tsc_init@Tce_standoff
    T1, N1 = get_T_N(Tse_init, Tse_standoff, speed, freq)
    traj1 = mr.ScrewTrajectory(Tse_init, Tse_standoff, T1, N1, method)
    traj1_csv = csv_format(traj1, gripper)

    # 2. Go to grasp position. Tse_grasp
    print("\nSTEP 2: Move to grasp")
    Tse_grasp = Tsc_init@Tce_grasp
    T2, N2 = get_T_N(Tse_standoff, Tse_grasp, speed, freq)
    traj2 = mr.CartesianTrajectory(Tse_standoff, Tse_grasp, T2*standoff_scale, N2*standoff_scale, method)
    traj2_csv = csv_format(traj2, gripper)

    # 3. Grasp
    print("\nSTEP 3: Grasp")
    gripper = 1
    T3 = round(gripper_close*1.5,2)
    N3 = int(T3*freq)
    traj3 = mr.CartesianTrajectory(Tse_grasp, Tse_grasp, T3, N3, method)
    traj3_csv = csv_format(traj3, gripper)

    # 4. Go back to standoff position Tse_standoff
    print("\nSTEP 4: Move back to standoff")
    T4, N4 = get_T_N(Tse_grasp, Tse_standoff, speed, freq)
    traj4 = mr.CartesianTrajectory(Tse_grasp, Tse_standoff, T4*standoff_scale, N4*standoff_scale, method)
    traj4_csv = csv_format(traj4, gripper)

    # 5. New standoff
    print("\nSTEP 5: Move to second standoff")
    Tse_standoff_final = Tsc_final@Tce_standoff
    T5, N5 = get_T_N(Tse_standoff, Tse_standoff_final, speed, freq)
    traj5 = mr.ScrewTrajectory(Tse_standoff, Tse_standoff_final, T5, N5, method)
    traj5_csv = csv_format(traj5, gripper)

    # 6. Go down
    print("\nSTEP 6: Place the brick down")
    Tse_grasp_final = Tsc_final@Tce_grasp
    T6, N6 = get_T_N(Tse_standoff_final, Tse_grasp_final, speed, freq)
    traj6 = mr.ScrewTrajectory(Tse_standoff_final, Tse_grasp_final, T6*standoff_scale, N6*standoff_scale, method)
    traj6_csv = csv_format(traj6, gripper)

    # 7. Ungrasp
    print("\nSTEP 7: Release gripper")
    gripper = 0
    T7 = round(gripper_close*1.5,2)
    N7 = int(T7*freq)
    traj7 = mr.CartesianTrajectory(Tse_grasp_final, Tse_grasp_final, T7, N7, method)
    traj7_csv = csv_format(traj7, gripper)

    # 8. Go back to standoff
    print("\nSTEP 8: Go back to standoff")
    T8, N8 = get_T_N(Tse_grasp_final, Tse_standoff_final, speed, freq)
    traj8 = mr.ScrewTrajectory(Tse_grasp_final, Tse_standoff_final, T8*standoff_scale, N8*standoff_scale, method)
    traj8_csv = csv_format(traj8, gripper)

    traj_csv = traj1_csv+traj2_csv+traj3_csv+traj4_csv+traj5_csv+traj6_csv+traj7_csv+traj8_csv
    np.savetxt('trajectory.csv', np.array(traj_csv), fmt='%10.5f', delimiter=',')
    return traj_csv

# Arm initial position
Tse_init = np.array([[0, 0, 1, 0],
                     [0, 1, 0, 0],
                     [-1, 0, 0, 0.5],
                     [0, 0, 0, 1]])

# Cube initial position
Tsc_init = np.array([[1, 0, 0, 1],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

# Cube final position
Tsc_final = np.array([[0, 1, 0, 0],
                      [-1, 0, 0, -1],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

# Angle defining the grasp orientation.
# Pi/2: goes in parallel to floor. 
# Add an additional pi/4 to have it go in at a 45 degree angle. 
theta = np.pi/2 + np.pi/4.

# Grasp position relative to cube: 0 offsett
Tce_grasp = np.array([[np.cos(theta), 0, np.sin(theta), 0],
                      [0, 1, 0, 0],
                      [-np.sin(theta), 0, np.cos(theta), 0.025],
                      [0, 0, 0, 1]])

# How far above the block the standoff position is
height = 0.1
Tce_standoff = np.array([[np.cos(theta), 0, np.sin(theta), 0],
                      [0, 1, 0, 0],
                      [-np.sin(theta), 0, np.cos(theta), 0.025+height],
                      [0, 0, 0, 1]])
                         
k = 1

# traj = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k)
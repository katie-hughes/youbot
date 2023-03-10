"""
GENERATE CSV WITH: 
python3 milestone2.py
"""

import modern_robotics as mr
import numpy as np
import matplotlib.pyplot as plt

np.set_printoptions(suppress=True)
np.set_printoptions(linewidth=np.inf)

###################################### SET PARAMS ##################################################

# To run a different config, change one of these to true
best = False
overshoot = False
newblock = False
new = True

# Default cube xy in s frame.
cube_initial = [1,0]
cube_final = [0,-1]

Kp = 2*np.identity(6)
Ki = 0.5*np.identity(6)

if newblock:
    print("New Block")
    cube_initial = [0.5, 0.5]
    cube_final = [0, -0.5]
    print(f"New starting block location: {cube_initial}")
    print(f"New ending block location: {cube_final}")

if overshoot:
    print("Overshoot")
    Kp = 170*np.identity(6)
    Ki = 0*np.identity(6)

if new:
    print("New")
    Kp = 0*np.identity(6)
    Ki = 100*np.identity(6)

if best:
    print("Best")
    # This is the default
    pass

# Starting configuration
theta2 = 0
theta3 = 0
theta4 = 0 # -1.57
config = [0,-0.5,-0.5,0,theta2,theta3,theta4,0,0,0,0,0,0]

print()

###################################### MILESTONE 1 #################################################

wheel_rad = 0.0475
l = 0.47/2.
w = 0.3/2.
# Got this from eq 13.10 from the textbook. Now realizing it is irrelevant :(
H0 = (1./wheel_rad) * np.array([[-l-w, 1, -1],
                                [l+w, 1, 1],
                                [l+w, 1, -1],
                                [-l-w, 1, 1]])

# Eq 13.33: Vb = F * deltaTheta
F = (wheel_rad/4.) * np.array([[-1./(l+w), 1./(l+w), 1./(l+w), -1./(l+w)],
                               [1, 1, 1, 1],
                               [-1, 1, -1, 1]])

# print(f"F: {F}")

M0e = np.array([[1, 0, 0, 0.033],
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]])

M0e_x = M0e[0][3]
M0e_z = M0e[2][3]

B1 = np.array([0, 0, 1, 0, 0.033, 0])
B2 = np.array([0, -1, 0, -0.5076, 0, 0])
B3 = np.array([0, -1, 0, -0.3526, 0, 0])
B4 = np.array([0, -1, 0, -0.2176, 0, 0])
B5 = np.array([0, 0, 1, 0, 0, 0])

Blist = np.array([B1, B2, B3, B4, B5]).T

Tb0 = np.array([[1, 0, 0, 0.1622],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0026],
                [0, 0, 0, 1]])

Tb0_x = Tb0[0][3]
Tb0_z = Tb0[2][3]

m0 = np.array([0,0,0,0])
F6 = np.vstack((m0, m0, F, m0))


def NextState(current_config, controls, dt, joint_threshold):
    """
    Steps the robot forward
    
    Args
        current_config: 13 vector representing current configuraiton of the robot.
            3 variables for the chassis configuration,
            5 variables for the arm configuration,
            4 variables for the wheel angles,
            1 for gripper open/closed
        controls: 9 vector indicating wheel speeds u (4 variables)
            and arm joint speeds thetadot (5 variables)
        dt: timestep (sec)
        joint_threshold: positive real value indicating the maximum angular speed
            of arm joints and wheels
    Returns:
        new_config: configuration after timestep dt based on euler step
    """
    # TODO: implement joint threshold
    # Convert to np array to make operations easier
    current_config = np.array(current_config)
    controls = np.array(controls)
    for i in range(len(controls)):
        if controls[i] > joint_threshold:
            # print("Speed too large")
            controls[i] = joint_threshold
        elif controls[i] < -joint_threshold:
            # print("Speed too small")
            controls[i] = -joint_threshold
    # Extract out specific configs
    current_chassis_config = current_config[:3]
    current_arm_config = current_config[3:8]
    current_wheel_config = current_config[8:12]
    current_gripper_config = [current_config[12]]
    # Extract out speeds
    wheel_speeds = controls[:4]
    arm_speeds = controls[4:]
    # Compute new configs
    new_arm_config = current_arm_config + arm_speeds*dt
    new_wheel_config = current_wheel_config + wheel_speeds*dt
    phi = current_chassis_config[0]
    delta_theta = wheel_speeds*dt
    Vb = F@delta_theta
    wbz = Vb[0]
    vbx = Vb[1]
    vby = Vb[2]
    # Follow eq. 13.35
    if wbz == 0:
        delta_qb = np.array([0, vbx, vby])
    else:
        delta_qb = np.array([wbz,
                            (vbx*np.sin(wbz) + vby*(np.cos(wbz)-1))/wbz,
                            (vby*np.sin(wbz) + vbx*(1-np.cos(wbz)))/wbz])
    delta_q = np.array([[1, 0, 0],
                        [0, np.cos(phi), -np.sin(phi)],
                        [0, np.sin(phi), np.cos(phi)]]) @ delta_qb
    new_chassis_config = current_chassis_config+delta_q
    # Combine everything back together
    return np.concatenate([new_chassis_config,
                           new_arm_config,
                           new_wheel_config,
                           current_gripper_config])


milestone1 = False
if milestone1:
    configs = []

    dt = 0.01
    config = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    configs.append(np.concatenate([config, [0]]))
    controls = [-10, 10, 10, -10, 0, 0, 0, 0, 0]
    joint_threshold = 100
    for i in range(0,100):
        config = NextState(config,controls,dt,joint_threshold)
        print(config)
        configs.append(np.concatenate([config, [0]]))

    np.savetxt('movement.csv', np.array(configs), fmt='%10.5f', delimiter=',')


###################################### MILESTONE 2 #################################################

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


def TrajectoryGenerator2(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k):
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
        traj : List of transformation matrices describing motion of end effector over trajectory
        grips: List of gripper state over trajectory
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
    # print(Tse_standoff)
    T1, N1 = get_T_N(Tse_init, Tse_standoff, speed, freq)
    traj1 = mr.ScrewTrajectory(Tse_init, Tse_standoff, T1, N1, method)
    grip1 = np.full(N1,gripper)

    # 2. Go to grasp position. Tse_grasp
    print("\nSTEP 2: Move to grasp")
    Tse_grasp = Tsc_init@Tce_grasp
    # print(Tse_grasp)
    T2, N2 = get_T_N(Tse_standoff, Tse_grasp, speed, freq)
    traj2 = mr.CartesianTrajectory(Tse_standoff, Tse_grasp, T2*standoff_scale, N2*standoff_scale, method)
    grip2 = np.full(N2*standoff_scale,gripper)

    # 3. Grasp
    print("\nSTEP 3: Grasp")
    gripper = 1
    T3 = round(gripper_close*1.5,2)
    N3 = int(T3*freq)
    traj3 = mr.CartesianTrajectory(Tse_grasp, Tse_grasp, T3, N3, method)
    grip3 = np.full(N3,gripper)

    # 4. Go back to standoff position Tse_standoff
    print("\nSTEP 4: Move back to standoff")
    T4, N4 = get_T_N(Tse_grasp, Tse_standoff, speed, freq)
    traj4 = mr.CartesianTrajectory(Tse_grasp, Tse_standoff, T4*standoff_scale, N4*standoff_scale, method)
    grip4 = np.full(N4*standoff_scale,gripper)

    # 5. New standoff
    print("\nSTEP 5: Move to second standoff")
    Tse_standoff_final = Tsc_final@Tce_standoff
    # print(Tse_standoff_final)
    T5, N5 = get_T_N(Tse_standoff, Tse_standoff_final, speed, freq)
    traj5 = mr.ScrewTrajectory(Tse_standoff, Tse_standoff_final, T5, N5, method)
    grip5 = np.full(N5,gripper)

    # 6. Go down
    print("\nSTEP 6: Place the brick down")
    Tse_grasp_final = Tsc_final@Tce_grasp
    # print(Tse_grasp_final)
    T6, N6 = get_T_N(Tse_standoff_final, Tse_grasp_final, speed, freq)
    traj6 = mr.ScrewTrajectory(Tse_standoff_final, Tse_grasp_final, T6*standoff_scale, N6*standoff_scale, method)
    grip6 = np.full(N6*standoff_scale,gripper)

    # 7. Ungrasp
    print("\nSTEP 7: Release gripper")
    gripper = 0
    T7 = round(gripper_close*1.5,2)
    N7 = int(T7*freq)
    traj7 = mr.CartesianTrajectory(Tse_grasp_final, Tse_grasp_final, T7, N7, method)
    grip7 = np.full(N7,gripper)

    # 8. Go back to standoff
    print("\nSTEP 8: Go back to standoff")
    T8, N8 = get_T_N(Tse_grasp_final, Tse_standoff_final, speed, freq)
    traj8 = mr.ScrewTrajectory(Tse_grasp_final, Tse_standoff_final, T8*standoff_scale, N8*standoff_scale, method)
    grip8 = np.full(N8*standoff_scale,gripper)

    traj = np.concatenate((traj1,traj2,traj3,traj4,traj5,traj6,traj7,traj8))
    grips = np.concatenate((grip1,grip2,grip3,grip4,grip5,grip6,grip7,grip8))
    # np.savetxt('trajectory.csv', np.array(traj_csv), fmt='%10.5f', delimiter=',')
    return traj,grips

# Arm initial position
Tse_init = np.array([[0, 0, 1, 0],
                     [0, 1, 0, 0],
                     [-1, 0, 0, 0.5],
                     [0, 0, 0, 1]])

# Cube initial position

Tsc_init = np.array([[1, 0, 0, cube_initial[0]],
                     [0, 1, 0, cube_initial[1]],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

# Cube final position
Tsc_final = np.array([[0, 1, 0, cube_final[0]],
                      [-1, 0, 0, cube_final[-1]],
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


###################################### MILESTONE 3 #################################################


def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt):
    """
    Feedforward control
    
    args:
        X: current actual end effector configuration
        Xd: current end effector reference configuration (desired)
        Xd_next: End effector reference configuration at the next timestep in the reference
            trajectory, at a time dt later.
        Kp: PI gain matrix
        Ki: PI gain matrix
        dt: timestep between reference trajectory configurations
    returns:
        V: commanded end effector twist in end effector frame
    """
    Vd = mr.se3ToVec((1./dt)*mr.MatrixLog6(mr.TransInv(Xd)@Xd_next))
    # Xerr extracted from log(X^-1 * Xd)
    Xerr = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(X)@Xd))
    feedforward = mr.Adjoint(mr.TransInv(X)@Xd)@Vd
    P_term = Kp@Xerr
    I_term = Ki@(Xerr*dt)
    V = feedforward + P_term + I_term
    return V, Xerr

milestone3 = False
if milestone3:
    # Kp = np.zeros((6,6))
    Kp = 20*np.identity(6)
    # Ki = np.zeros((6,6))
    Ki = 5*np.identity(6)

    # config: phi, x, y, theta1-5
    config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
    x = config[1]
    y = config[2]
    phi = config[0]
    thetalist = config[3:]
    print(f"Thetalist:{thetalist}")
    Tsb = np.array([[np.cos(phi), np.sin(phi), 0, x],
                        [np.sin(phi), np.cos(phi), 0, y],
                        [0, 0, 1, 0.0963],
                        [0, 0, 0, 1]])
    start_T = np.array(Tsb)
    start_T[0][3] += (Tb0_x+M0e_x)
    start_T[2][3] += (Tb0_z+M0e_z)
    X=mr.FKinBody(start_T, Blist, thetalist)
    print(f"X:\n{X}")

    Xd = np.array([[0, 0, 1, 0.5],
                   [0, 1, 0, 0],
                   [-1, 0, 0, 0.5],
                   [0, 0, 0, 1]])
    Xd_next = np.array([[0, 0, 1, 0.6],
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0.3],
                        [0, 0, 0, 1]])

    print("\n\nCONTROLS")
    V, Xerr = FeedbackControl(X, Xd, Xd_next, Kp, Ki, 0.01)

    Jacobian_arm = mr.JacobianBody(Blist, thetalist)
    print(f"Jacobian arm:\n{Jacobian_arm}")
    print(Jacobian_arm.shape)

    T0e = mr.FKinBody(M0e, Blist, thetalist)

    Jacobian_base = mr.Adjoint(mr.TransInv(T0e)@mr.TransInv(Tb0))@F6
    print(f"Jacobian base:\n{Jacobian_base}")
    print(Jacobian_base.shape)

    Jacobian = np.hstack((Jacobian_base, Jacobian_arm))
    print(f"Jacobian\n{Jacobian}")
    print(Jacobian.shape)

    Je_pseudoinverse = np.linalg.pinv(Jacobian)
    print(f"PSEUDOINVERSE:\n{Je_pseudoinverse}")
    new_speeds = Je_pseudoinverse @ V
    print(f"New Speeds:\n{new_speeds}")

###################################### COMBINED ####################################################

def get_X(config):
    # Take in 13 vector return X
    x = config[1]
    y = config[2]
    phi = config[0]
    thetalist = config[3:8]
    # This negative sign took me 4 hours to debug :)
    Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                    [np.sin(phi), np.cos(phi), 0, y],
                    [0, 0, 1, 0.0963],
                    [0, 0, 0, 1]])
    T0e = mr.FKinBody(M0e, Blist, thetalist)
    # From pg 548 of book
    X = Tsb@Tb0@T0e
    return X

dt = 0.01
joint_threshold = 100




def deg_to_rad(deg):
    return deg*(np.pi/180.)

# I took these from the YouBot datasheet
# joint_limits_lower = [-169, -65, -150, -102.5, -167.5]
# joint_limits_upper = [169,   90,  146,  102.5,  167.5]
# joint_limits_lower = [deg_to_rad(i) for i in joint_limits_lower]
# joint_limits_upper = [deg_to_rad(i) for i in joint_limits_upper]

# joint_limits_lower = [None,  None, None, None, None]
# joint_limits_upper = [None,   0.5,  -0.2,  -0,2,  None]

# I could not get this to reliably work so I have no joint limits :(
joint_limits_lower = [None,  None, None, None, None]
joint_limits_upper = [None,  None, None, None,  None]

print(f"Lower Joint Limits: {joint_limits_lower}")
print(f"Upper Joint Limits: {joint_limits_upper}")

def TestJointLimits(config):
    violated = [False, False, False, False, False]
    for j in range(0,5):
        # print(config[3+j])
        if joint_limits_lower[j]:
            if config[3+j]<joint_limits_lower[j]:
                print(f"Joint {j} too low")
                config[3+j] = joint_limits_lower[j]
                violated[j] = True
        if joint_limits_upper[j]:
            if config[3+j]>joint_limits_upper[j]:
                print(f"Joint {j} too high")
                config[3+j] = joint_limits_upper[j]
                violated[j] = True
    return violated


print('\nGenerating my reference trajectory:\n')
traj,grips = TrajectoryGenerator2(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k)
n = len(traj)

saved_configs = []
xerrs = []
print(f"Starting config:\n{config}")
print("\nGenerating my trajectory based on reference trajectory.\n")
saved_configs.append(config)
for i in range(0,n-1):
    # Find Xd, Xd_next from generated trajectory
    Xd = traj[i]
    Xd_next = traj[i+1]
    # Find X of current config.
    X = get_X(config)
    # Calculate new twist based on these Xs
    V, Xerr = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt)
    thetalist = config[3:8]
    Jacobian_arm = mr.JacobianBody(Blist, thetalist)
    # Test joint limits
    violated = TestJointLimits(config)
    # Change jacobian arm if any limits violated
    if any(violated):
        for j,v in enumerate(violated):
            if v:
                print(f"Change jacobian col {j}")
                Jacobian_arm[:, j] = np.full(6,1e-3)
    T0e = mr.FKinBody(M0e, Blist, thetalist)
    Jacobian_base = mr.Adjoint(mr.TransInv(T0e)@mr.TransInv(Tb0))@F6
    Jacobian = np.hstack((Jacobian_base, Jacobian_arm))
    Je_pseudoinverse = np.linalg.pinv(Jacobian)
    controls = Je_pseudoinverse @ V
    # Go to next state. Update config with NextState. Update grip state too.
    config[12] = grips[i]
    config = NextState(config, controls, dt, joint_threshold)
    # print(f"{i}\t{config}")
    saved_configs.append(config)
    xerrs.append(Xerr)

print("\nDone! Saving .csv files.\n")

np.savetxt('configs.csv', np.array(saved_configs), fmt='%10.5f', delimiter=',')

xerrs = np.array(xerrs).T

np.savetxt('errors.csv', xerrs.T, fmt='%10.5f', delimiter=',')

fig, axs = plt.subplots(2, 1)

axs[0].plot(xerrs[0],label='Roll err')
axs[0].plot(xerrs[1],label='Pitch err')
axs[0].plot(xerrs[2],label='Yaw err')
axs[0].legend()
axs[0].set(xlabel='iteration',ylabel='radians')
axs[0].set_title("Angular Error")

axs[1].plot(xerrs[3],label='X err')
axs[1].plot(xerrs[4],label='Y err')
axs[1].plot(xerrs[5],label='Z err')
axs[1].legend()
axs[1].set(xlabel='iteration',ylabel='m')
axs[1].set_title("Linear Error")

fig.tight_layout()
plt.savefig('errors.png')
plt.show()
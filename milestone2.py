import modern_robotics as mr
import numpy as np


def csv_format(traj, gripper):
    res = []
    for i in traj:
        newline = [i[0][0], i[0][1], i[0][2],
                   i[1][0], i[1][1], i[1][2],
                   i[2][0], i[2][1], i[2][2],
                   i[0][3], i[1][3], i[2][3],
                   gripper]
        # print()
        # print(i)
        # print(newline)
        res.append(newline)
    return res

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
    # 1. Tse_init to Tce_standoff
    Tf = 5
    N = 100
    method = 5
    gripper = 0
    # Want: Standoff position in s frame. Tse_standoff
    Tse_standoff = Tsc_init@Tce_standoff
    traj1 = mr.ScrewTrajectory(Tse_init, Tse_standoff, Tf, N, method)
    traj1_csv = csv_format(traj1, gripper)

    # 2. Go to grasp position. Tse_grasp
    Tse_grasp = Tsc_init@Tce_grasp
    traj2 = mr.CartesianTrajectory(Tse_standoff, Tse_grasp, Tf, N, method)
    traj2_csv = csv_format(traj2, gripper)

    # 3. Grasp
    gripper = 1

    # 4. Go back to standoff position Tse_standoff
    traj4 = mr.CartesianTrajectory(Tse_grasp, Tse_standoff, Tf, N, method)
    traj4_csv = csv_format(traj4, gripper)

    # 5. New standoff
    Tse_standoff_final = Tsc_final@Tce_standoff
    traj5 = mr.ScrewTrajectory(Tse_standoff, Tse_standoff_final, Tf, N, method)
    traj5_csv = csv_format(traj5, gripper)

    # 6. Go down
    Tse_grasp_final = Tsc_final@Tce_grasp
    traj6 = mr.ScrewTrajectory(Tse_standoff_final, Tse_grasp_final, Tf, N, method)
    traj6_csv = csv_format(traj6, gripper)

    # 7. Ungrasp
    gripper = 0

    # 8. Go back to standoff
    traj7 = mr.ScrewTrajectory(Tse_grasp_final, Tse_standoff_final, Tf, N, method)
    traj7_csv = csv_format(traj7, gripper)

    traj_csv = traj1_csv+traj2_csv+traj4_csv+traj5_csv+traj6_csv+traj7_csv
    np.savetxt('test.csv', np.array(traj_csv), fmt='%10.5f', delimiter=',')
    return traj2

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

# Grasp position relative to cube: 0 offsett
Tce_grasp = np.array([[0, 0, 1, 0],
                      [0, 1, 0, 0],
                      [-1, 0, 0, 0.025],
                      [0, 0, 0, 1]])

# Standoff position above cube
# later: change angle to 45 deg so it doesn't go in straight
Tce_standoff = np.array([[0, 0, 1, 0],
                         [0, 1, 0, 0],
                         [-1, 0, 0, 0.2],
                         [0, 0, 0, 1]])

k = 1
res = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k)
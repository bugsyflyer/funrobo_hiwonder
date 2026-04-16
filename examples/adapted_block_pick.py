import time
import numpy as np
from math import sin, cos, atan, acos, asin, sqrt, atan2, pi
import traceback
import os
import sys

from funrobo_hiwonder.core.hiwonder import HiwonderRobot

#from funrobo_kinematics.core.arm_models import FiveDOFRobotTemplate
sys.path.append("../..")
import funrobo_kinematics.funrobo_kinematics.core.utils as ut
from funrobo_kinematics.examples.traj_gen import MultiAxisTrajectoryGenerator
from funrobo_kinematics.examples.hiwonder import FiveDOFRobot
from funrobo_hiwonder.core.hiwonder import HiwonderRobot

# Initialize components
robot = HiwonderRobot()
model = FiveDOFRobot()

PI = 3.1415926535897932384
K2 = [1, 0, 0]
K_VEC = [0, 0, 1]  # Used in isolating the Z component of transformation matrices
DET_J_THRESH = 3 * 10 & -5  # Threshold for when determinant is "close to 0"
VEL_SCALE = 0.25  # Scale the EE velocity by this factor when close to a singularity

# Point where robot will try to drop cube into bucket
DROP_POINT = ut.EndEffector(
    x=-13.492, y=2.384, z=17.884, rotx=2.967, roty=-0.676, rotz=3.142
)

soln = 0

def collect_cube_traj(obj: ut.Position, pos: ut.Position, pitch: float):
        """
        Given an object position, end effector position, and end effector pitch,
        generate a trajectory to the object, collect it, and deposit it in the
        bin on the back of the robot.

        Args:
            obj (ut.Position): Position of the object in the base frame
            pos (ut.Position): Position of the end effector in the base frame
            pitch (float): Current pitch of the end effector
        """

        obj.x *= 1.1
        obj.x += 1
        object_d = np.sqrt(obj.x**2 + obj.y**2)
        print("dist", object_d)
        pitch += (object_d - 15) * 0.06
        if pitch > 0.8:
            pitch = 0.8
        elif pitch < 0:
            pitch = 0
        print("pitch", pitch)
        d_adjust = (object_d - 4) / object_d
        int_position_1 = ut.Position(x=obj.x * d_adjust, y=obj.y * d_adjust, z=pos.z)
        int_position_2 = ut.Position(x=obj.x * d_adjust, y=obj.y * d_adjust, z=obj.z)

        # Compile waypoints
        waypoints = [pos, int_position_2, obj]

        # Do trajectory generation
        task_space_traj(waypoints, pitch)
        time.sleep(2)
        close_pos = joint_values
        close_pos[5] = -15
        set_joint_values(close_pos, duration=100, radians=False)
        time.sleep(2)
        FiveDOFRobot.calc_inverse_kinematics(DROP_POINT, duration=1500)
        time.sleep(2)
        open_pos = joint_values
        open_pos[5] = -70
        set_joint_values(open_pos, duration=70, radians=False)
        time.sleep(2)
        move_to_home_position()
        time.sleep(2)
        
def analytical_ik(self, EE: ut.EndEffector, duration=700):
    """
    Use analytical inverse kinematics to calculate the joint angles for a
    given end effector position and move to that position.

    Args:
        EE (ut.EndEffector): Desired position of end effector
        duration (int, optional): Desired duration of movement. Defaults to 700.
    """

    x, y, z = EE.x, EE.y, EE.z
    print("xyzrpy", x, y, z, EE.rotx, EE.roty, EE.rotz)
    R_05 = ut.euler_to_rotm((EE.rotx, EE.roty, EE.rotz))
    R_05_K = R_05 @ K_VEC

    p_wrist = [x, y, z] - ((l4 + l5) * (R_05_K))

    wx = p_wrist[0]
    wy = p_wrist[1]
    wz = p_wrist[2]

    r = sqrt((wx**2 + wy**2) + ((wz - l1) ** 2))

    # Theta 1 standard solve
    j1 = ut.wraptopi(atan2(y, x) + np.pi)  # seems to give desired - 180

    # Theta 2 standard solve
    alpha = acos((r**2 + l2**2 - l3**2) / (2 * r * l2))
    phi = acos((wz - l1) / (r))
    j2 = alpha + phi

    # Theta 3 standard solve
    j3 = acos((r**2 - l2**2 - l3**2) / (2 * l2 * l3))

    # Set of 4 potential solutions
    solns = np.array(
        [
            # Standard configuration
            [j1, j2, j3, 0, 0],
            # Flipped elbow
            [j1, -alpha + phi, -j3, 0, 0],
            # Mirrored base
            [ut.wraptopi(j1 + PI), -alpha - phi, -j3, 0, 0],
            # Mirrored base, flipped elbow
            [ut.wraptopi(j1 + PI), alpha - phi, j3, 0, 0],
        ]
    )

    # Keep track of how many valid solutions have been "skipped"
    valid_solns_count = 0

    new_thetalist = np.deg2rad(joint_values)

    for angs in solns:
        # print(angs)
        # Temporary DH matrix
        mini_DH = [
            [angs[0], l1, 0, np.pi / 2],
            [(np.pi / 2) + angs[1], 0, l2, 0],
            [-angs[2], 0, l3, 0],
        ]

        # Translation matrix from 0 to 3
        t_03 = np.eye(4)
        for dh_item in mini_DH:
            t_temp = ut.dh_to_matrix(dh_item)
            t_03 = t_03 @ t_temp

        # Rotation matrix from 0 to 3
        R_03 = t_03[:3, :3]

        # Rotation matrix from 3 to 5
        R_35 = np.transpose(R_03) @ R_05

        # Solve for Theta 4 and Theta 5
        angs[3] = atan2(R_35[1, 2], R_35[0, 2])
        angs[4] = ut.wraptopi(atan2(R_35[2, 0], R_35[2, 1]) + PI)

        # Check if the current configuration is valid
        # print(angs)
        if ut.check_joint_limits(angs, np.deg2rad(joint_limits)):
            # Check if we've reached either the requested `soln` or the last valid solution
            if soln is valid_solns_count or valid_solns_count is len(solns) - 1:
                new_thetalist[0:5] = angs
                break
            # "Skip" valid solutions until the requested `soln`
            else:
                last_valid = angs  # Hang onto the most recent valid solution
                valid_solns_count += 1
        # If we've made it to the end and don't have another valid solution, use the most recent valid one
        elif valid_solns_count is len(solns) - 1:
            new_thetalist[0:5] = last_valid
    new_thetalist[5] = np.deg2rad(joint_values[5])
    print("new theta", new_thetalist)
    new_thetalist = enforce_joint_limits(new_thetalist, radians=True)
    print("new theta", new_thetalist)
    print()
    set_joint_values(new_thetalist, duration=duration, radians=True)
        
def move_to_home_position(self):
        print(f"Moving to home position...")
        set_joint_values(home_position, duration=2000)
        time.sleep(2.0)
        print(f"Arrived at home position: {joint_values} \n")
        time.sleep(1.0)
        
def task_space_traj(self, waypoints: list[ut.Position], pitch: float, nsteps=10):
        """
        Given a list of desired waypoints, generate a trajectory to follow them in task space.

        Args:
            waypoints (list[ut.Position]): List of desired trajectory waypoint positions.
            pitch (float): Current pitch of end effector.
            nsteps (int, optional): Desired number of trajectory points. Defaults to 10.
        """
        traj_dofs = None
        for i in range(len(waypoints) - 1):
            start_vel = None
            final_vel = None
            start_acc = None
            final_acc = None

            traj = MultiAxisTrajectoryGenerator(
                method="quintic",
                mode="task",
                interval=[0, 1],
                ndof=len(np.array(waypoints[0])),
                start_pos=np.array(waypoints[i]),
                final_pos=np.array(waypoints[i + 1]),
                start_vel=start_vel,
                final_vel=final_vel,
                start_acc=start_acc,
                final_acc=final_acc,
            )
            points = traj.generate(nsteps=nsteps)
            if traj_dofs is not None:
                traj_dofs = np.concatenate([traj_dofs, points], axis=2)
            else:
                traj_dofs = points

        for i in range(nsteps * (len(waypoints) - 1)):
            pos = [dof[0][i] for dof in traj_dofs]
            ee = ut.EndEffector(
                x=pos[0],
                y=pos[1],
                z=pos[2],
                rotx=ut.wraptopi(atan2(pos[1], pos[0]) + PI),
                roty=pitch,
                rotz=PI,
            )
            analytical_ik(ee)
            time.sleep(0.5)  # TODO will prob need to adjust
        print("DONE")
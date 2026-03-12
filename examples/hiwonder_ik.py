# main.py
"""
Main Application Script
----------------------------
Example code for the MP1 RRMC implementation
"""

import time
import traceback
import os
import sys
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../funrobo_hiwonder")))

from funrobo_kinematics.scripts.FiveDOF_rrmc import FiveDOF

from funrobo_hiwonder.core.hiwonder import HiwonderRobot

#from funrobo_kinematics.core.arm_models import FiveDOFRobotTemplate
import funrobo_kinematics.funrobo_kinematics.core.utils as ut



def main():
    """ Main loop that reads gamepad commands and updates the robot accordingly. """
    robot = None
    try:

        # Initialize components
        robot = HiwonderRobot()
        model = FiveDOF()
        
        control_hz = 20 
        #dt = 1 / control_hz
        dt = 2
        t0 = time.time()
        curr_joint_values = robot.get_joint_values()
        new_joint_values = [np.deg2rad(theta) for theta in curr_joint_values]
        #joint_values = np.deg2rad([0.373, -13.448, 75.456, -30, 0])
        #ee, _ = model.calc_forward_kinematics(new_joint_values[:5])
        # waypoints1 = [[-0.2581, 0, 0.31],
        #             [-0.2581, 0, 0.06+0.2258],
        #             [-0.2581, -.06, 0.06+0.2258],
        # ]
        
        #.31 corresponds with top
        star_waypoints = [[-0.2581, 0.0, .33],
                     [-0.2581, -.06, .27],
                     [-0.2581, -.038, .18],
                     [-0.2581, .038, .18],
                     [-0.2581, .063, .28],
                     [-0.2581, 0.0, .33]]
        
        square_waypoints = [[-0.2581, -.06, .31],
                            [-0.2581, -.06, .18],
                            [-0.2581, .063, .18],
                            [-0.2581, .063, .31],
                            [-0.2581, -.06, .31]]
        
        HI_waypoints = [[-0.2581, -.07, .3],
                        [-0.2581, -.07, .18],
                        [-0.2581, -.07, .27],
                        [-0.2581, 0.0, .27],
                        [-0.2581, 0.0, .18],
                        [-0.2581, 0.0, .3],
                        [-0.2581, .05, .3],
                        [-0.2581, .05, .18]]

        for point in HI_waypoints:
            ee = ut.EndEffector()
            ee.x = point[0]
            ee.y = point[1]
            ee.z = point[2]
            joint_values = model.calc_numerical_ik(ee, new_joint_values[:5])
            # set new joint angles
            print(f"Moving to: {point}")
            print(f"Calculated joint values (rad): {joint_values}")
            joint_values = list(joint_values) + [new_joint_values[5]]
            robot.set_joint_values(joint_values, duration=1, radians=True)
            time.sleep(dt)
            
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard Interrupt detected. Initiating shutdown...")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        traceback.print_exc()
    finally:
        if robot:
            robot.shutdown_robot()




if __name__ == "__main__":
    main()
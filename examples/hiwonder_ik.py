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

sys.path.append("../..")

from funrobo_kinematics.scripts.FiveDOF_rrmc import FiveDOFRobot

from funrobo_hiwonder.core.hiwonder import HiwonderRobot

#from funrobo_kinematics.core.arm_models import FiveDOFRobotTemplate
import funrobo_kinematics.funrobo_kinematics.core.utils as ut



def main():
    """ Main loop that reads gamepad commands and updates the robot accordingly. """
    try:

        # Initialize components
        robot = HiwonderRobot()
        model = FiveDOFRobot()
        
        control_hz = 20 
        #dt = 1 / control_hz
        dt = 2
        t0 = time.time()
        curr_joint_values = robot.get_joint_values()
        new_joint_values = curr_joint_values.copy()
        point1 = [0,0,0]
        point2 = [0.2, 0.4, 0.5]
        waypoints = [point1, point2]
        for point in waypoints:
            new_joint_values = model.calc_numeric_ik(new_joint_values)
            # set new joint angles
            robot.set_joint_values(new_joint_values, duration=dt, radians=False)
            
        elapsed = time.time() - t_start
        remaining_time = dt - elapsed
        if remaining_time > 0:
            time.sleep(remaining_time)


            
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard Interrupt detected. Initiating shutdown...")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        traceback.print_exc()
    finally:
        robot.shutdown_robot()




if __name__ == "__main__":
    main()



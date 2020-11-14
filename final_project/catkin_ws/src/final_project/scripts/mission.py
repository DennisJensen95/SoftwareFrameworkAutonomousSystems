#!/usr/bin/env python
import rospy
from mission_lib.robot_utilities import BurgerUtility
from mission_lib.qr_code_utilities import QrCodeUtility
import time

def main():
    # Initialize utilities
    burger = BurgerUtility()
    qr_code = QrCodeUtility()

    start = time.time()
    while (time.time() - start <= 3):
        print(burger.get_robot_x_y_position())
    
    rospy.signal_shutdown("Mission done")

    

if __name__ == "__main__":
    main()
#!/usr/bin/env python3

"""
Node runner for /Classes/robot_move_with_ur_rtde.py
@Depreciated name: robot_move_with_ik_node.py
"""

import rospy
from Classes.robot_move_with_ur_rtde import RobotCommander
from math import pi


def main(): 
	Robot = RobotCommander(start_node=True)
	Robot.init_subscribers_and_publishers()
	try:
		while not rospy.is_shutdown():
			Robot.update()
			Robot.r.sleep()
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise


if __name__ == '__main__': main()
#!/usr/bin/env python3

"""
Node runner for /Classes/robot_move_with_ur_rtde.py
@Depreciated name: robot_move_with_ik_node.py
"""

import rospy,sys,numpy
from Classes.robot_move_with_ur_rtde_with_TO import RobotCommander
from math import pi


def main(): 
	Robot = RobotCommander(start_node=True)
	Robot.init_subscribers_and_publishers()
	x = numpy.arange(-0.707,0.707,0.005)
	Robot.rtde_c.servoL([Robot.robot_init[0], Robot.robot_init[1], Robot.robot_init[2], Robot.robot_init[3], Robot.robot_init[4], Robot.robot_init[5]-0.707], 0.5, 0.3, 0.002, 0.1, 300)
	count = 0
	reverse = False
	try:
		while not rospy.is_shutdown():
			# Robot.update()
			if count == len(x)-1:
				reverse = True
			elif count == 0:
				reverse = False
			
			if not reverse:
				count += 1
			else:
				count-=1

			print(count)
			
			Robot.update2(x[count])
			Robot.r.sleep()
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise


if __name__ == '__main__': main()
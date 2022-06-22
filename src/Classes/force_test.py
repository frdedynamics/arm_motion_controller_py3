#!/usr/bin/env python3

# imports
import rospy
from math import radians as d2r
from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive


class RobotPoses:
	def __init__(self, rtde_c, rtde_r, mode="auto"):
		"""
		Takes the current robot connection if there is set one
		@params rtde_c: RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
		@params rtde_r: rtde_receive.RTDEReceiveInterface("172.31.1.144")
		@params mode: "manual" for user entry mode
		"""
		self.rtde_c = rtde_c
		self.rtde_r = rtde_r

		rospy.init_node("imu_subscriber_2arms")
		self.r = rospy.Rate(10)


		print("initialized")

		if mode == "manual":
			vector = self.rtde_r.getActualTCPPose()
			selection_vector = [1, 0, 0, 0, 0, 0]
			wrench = [20.0, 0.0, 0.0, 0.0, 0.0, 0.0]
			type = 2
			limits = [0.5, 0.1, 0.1, 0.17, 0.17, 0.17]
			self.rtde_c.forceMode(vector, selection_vector, wrench, type, limits)




if __name__ == '__main__':
	RobotPoses(RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP), rtde_receive.RTDEReceiveInterface("172.31.1.144"), mode="manual")

	while not rospy.is_shutdown():
		print("press enter to stop...")
		input()
		
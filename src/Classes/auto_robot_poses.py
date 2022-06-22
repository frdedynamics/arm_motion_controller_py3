#!/usr/bin/env python3

# imports
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

		# self.release_prev_joints = [None] * 6
		# self.release_approach_joints = [None] * 6
		# self.release_joints = [None] * 6
		# self.home_approach_joints = [None] * 6

		self.release_prev_joints = [d2r(-149.60), d2r(-58.63), d2r(59.90), d2r(7.99), d2r(41.82), d2r(-97.12)]
		self.release_approach_joints = [d2r(-158.48), d2r(-61.75), d2r(87.86), d2r(-14.92), d2r(33.14), d2r(-99.55)]
		self.release_joints = [d2r(-147.37), d2r(-51.78), d2r(71.37), d2r(-10.75), d2r(44.05), d2r(-96.53)]
		self.home_approach_joints = [d2r(-95.72), d2r(-85.83), d2r(95.93), d2r(-3.86), d2r(95.29), d2r(-89.49)]

		print("initialized")

		if mode == "manual":
			self.rtde_c.teachMode()
			print("Move the robot to RELEASE pose and then press Enter")
			input()
			self.release_joints = self.rtde_r.getActualQ()
			print("RELEASE: ", self.release_joints)

			print("Move the robot to RELEASE APPROACH pose and then press Enter")
			input()
			self.release_approach_joints = self.rtde_r.getActualQ()
			print("RELEASE APPROACH: ", self.release_approach_joints)

			print("The robot WILL MOVE TO RELEASE pose again. Press Enter to continue...")
			input()
			self.rtde_c.endTeachMode()
			self.rtde_c.moveJ(self.release_joints)
			self.rtde_c.teachMode()

			print("Move the robot to RELEASE PREVIOUS pose and then press Enter")
			input()
			self.release_prev_joints = self.rtde_r.getActualQ()
			print("RELEASE PREV: ", self.release_prev_joints)

			print("Move the robot to HOME APPROACH pose and then press Enter")
			input()
			self.home_approach_joints = self.rtde_r.getActualQ()
			print("HOME APPROACH: ", self.home_approach_joints)
			self.rtde_c.endTeachMode()


if __name__ == '__main__':
    RobotPoses(RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP), rtde_receive.RTDEReceiveInterface("172.31.1.144"), mode="manual")
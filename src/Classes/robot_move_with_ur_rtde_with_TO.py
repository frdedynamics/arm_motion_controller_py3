#! /usr/bin/env python3

"""
Subscribes two hand poses and drives the real UR5e robot in real-time.
"""
from os import stat
import sys
import rospy
from math import pi
from math import radians as d2r
import numpy as np

import actionlib
from arm_motion_controller_py3.msg import handCalibrationAction, handCalibrationGoal

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int16, Float64, Bool
from std_msgs.msg import Float32MultiArray

from . import Kinematics_with_Quaternions as kinematic

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive


class RobotCommander:
	def __init__(self, rate=100, start_node=False, sr=1.0, sl=1.0, so=2.0):
		"""Initializes the robot commander
			@params s: motion hand - steering hand scale
			@params k: target hand pose - robot pose scale"""
		# self.rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
		# self.rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")
		# self.rtde_c.moveL(self.home_teleop)
		print("Fake controller created")

		if start_node == True:
			rospy.init_node("robot_move_with_ur_rtde")
			self.r = rospy.Rate(rate)
			print("robot_move_with_ur_rtde Node Created")


		# TODO: Fill missing prefedined poses
		# self.robot_init = self.rtde_r.getActualTCPPose()
		self.robot_current_TCP = Float32MultiArray()
		self.release_before = []
		self.release_after = []
		self.release = []
		self.home_teleop_approach = []
		self.home_teleop = []
		self.home_hrc = []
		self.robot_colift_init = []

		# print("============ Arm current pose: ", self.rtde_r.getActualTCPPose())
		self.target_pose = Pose()
		self.left_hand_pose = Pose()
		self.hand_grip_strength = Int16()
		self.right_hand_pose = Pose()
		self.robot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

		self.target_pose_colift_init = Pose()
		self.motion_hand_colift_init = Pose()
		self.motion_hand_colift_pos_ch = Point()

		self.hand_init_orientation = Quaternion()
		self.human_to_robot_init_orientation = Quaternion(0.0, 0.0, 0.707, 0.707)
		self.sr = sr # WS scaling right hand
		self.sl = sl # WS scaling left hand 
		self.so = so # Orientation scaling of left wrist to robot wrist

		self.state = "IDLE"
		self.role = "HUMAN_LEADING"  # or "ROBOT_LEADING"
		self.hrc_status = String()

		self.wrist_calib_flag = False
		self.tcp_ori = Vector3()
		self.tcp_ori_init = Vector3()
               

	def init_subscribers_and_publishers(self):
		self.sub_hand_grip_strength = rospy.Subscriber('/robotiq_grip_gap', Int16, self.cb_hand_grip_strength)
		self.sub_left_hand_pose = rospy.Subscriber('/left_hand_pose', Pose, self.cb_left_hand_pose)
		self.sub_right_hand_pose = rospy.Subscriber('/right_hand_pose', Pose, self.cb_right_hand_pose)
		self.sub_human_ori = rospy.Subscriber('/human_ori', Quaternion, self.cb_human_ori)
		self.sub_sensor_lw = rospy.Subscriber('/sensor_l_wrist_rpy', Vector3, self.cb_sensor_lw)
		self.pub_tee_goal = rospy.Publisher('/Tee_goal_pose', Pose, queue_size=1)
		self.pub_hrc_status = rospy.Publisher('/hrc_status', String, queue_size=1)
		self.pub_grip_cmd = rospy.Publisher('/cmd_grip_bool', Bool, queue_size=1)


	####### Callback methods #######

	def cb_sensor_lw(self, msg):
		""" Subscribes the pure IMU RPY topic to control robot TCP orientation"""
		if not self.wrist_calib_flag:
			self.tcp_ori_init.x = msg.x
			self.tcp_ori_init.y = msg.y
			self.tcp_ori_init.z = msg.z
			self.wrist_calib_flag = True
		self.tcp_ori.x = d2r(msg.y-self.tcp_ori_init.y)
		self.tcp_ori.z = 0
		self.tcp_ori.y = 0  ## This didn't give good results


	def cb_human_ori(self, msg):
		""" Subscribes chest IMU orientation to map human w.r.t the world frame """
		self.human_to_robot_init_orientation = kinematic.q_multiply(Quaternion(0.0, 0.0, 0.707, 0.707), msg)


	def cb_hand_grip_strength(self, msg):
		""" Subscribes hand grip strength
		Open: 0
		Close: 255 """
		self.hand_grip_strength = msg

	def cb_left_hand_pose(self, msg):
		""" Subscribes left hand pose """
		self.left_hand_pose = msg	

	def cb_right_hand_pose(self, msg):
		""" Subscribes right hand pose """
		self.right_hand_pose = msg

	def call_hand_calib_server(self):

		self.client = actionlib.SimpleActionClient('hand_calibration_as', handCalibrationAction)

		self.client.wait_for_server()

		self.goal = handCalibrationGoal()
		self.goal.calib_request = True

		self.client.send_goal(self.goal, feedback_cb=self.hand_calib_feedback_cb)

		self.client.wait_for_result()

		result = self.client.get_result()

		return result # maybe result is not needed?
	
	def hand_calib_feedback_cb(self, msg):
		print('Hand poses initialized:', msg)


	####### State methods #######

	def teleop_idle(self):
		self.robot_pose = self.home_teleop
		self.rtde_c.moveJ_IK(self.robot_pose)
		if (self.right_hand_pose.orientation.w < 0.707 and self.right_hand_pose.orientation.x > 0.707):
			self.status = 'TO/active'
		else:
			self.status = 'TO/idle'
		return self.status


	def teleop_active(self):	
		self.target_pose.position.x = - self.s * self.right_hand_pose.position.x
		self.target_pose.position.y = - self.s * self.right_hand_pose.position.y
		self.target_pose.position.z = self.s * self.right_hand_pose.position.z

		corrected_target_pose = kinematic.q_rotate(self.human_to_robot_init_orientation, self.target_pose.position)
		self.robot_pose[0] = self.home_teleop[0] + self.k * corrected_target_pose[0]
		self.robot_pose[1] = self.home_teleop[1] - self.k * corrected_target_pose[1]
		self.robot_pose[2] = self.home_teleop[2] + self.k * corrected_target_pose[2]
		self.robot_pose[3] = self.home_teleop[3]
		self.robot_pose[4] = self.home_teleop[4]
		self.robot_pose[5] = self.home_teleop[5] + self.so*self.tcp_ori.x

		if (self.right_hand_pose.orientation.w < 0.707 and self.right_hand_pose.orientation.x > 0.707): # right rotate upwards
			self.status = 'TO/idle'
		# TODO: check the tresholds
		elif ((self.tcp_ori.x > 10.0) and (self.right_hand_pose.orientation.w > 0.707 and self.right_hand_pose.orientation.x < 0.707)):
			self.status = 'HRC/idle'
		else:
			self.status = 'TO/active'

		return self.status


	def hrc_idle(self, from_colift=False):
		if not from_colift:
			self.robot_pose = self.home_hrc
			self.rtde_c.moveJ_IK(self.robot_pose)  # or check with moveL
		# TODO: check the tresholds
		if((self.tcp_ori.x < 10.0) and (self.right_hand_pose.orientation.w < 0.707 and self.right_hand_pose.orientation.x > 0.707)):
			self.rtde_c.moveL(self.home_teleop)
			self.status = 'TO/active'
		elif(self.right_hand_pose.orientation.w < 0.707 and self.right_hand_pose.orientation.x > 0.707):
			# self.home_hrc_approach = self.rtde_r.getActualTCPPose()
			left, right = self.call_hand_calib_server()
			self.status = 'HRC/approach'
		else:
			self.status = 'HRC/idle'
		return self.status

	def hrc_approach(self):
		''' old cartesian_2_arms here 
		robot.init is updated to home_hrc'''
		self.target_pose.position.x = (- self.left_hand_pose.position.x) - self.s * self.right_hand_pose.position.x
		self.target_pose.position.y = (- self.left_hand_pose.position.y) - self.s * self.right_hand_pose.position.y
		self.target_pose.position.z = self.left_hand_pose.position.z + self.s * self.right_hand_pose.position.z
		self.target_pose.orientation = self.left_hand_pose.orientation

		corrected_target_pose = kinematic.q_rotate(self.human_to_robot_init_orientation, self.target_pose.position)
		self.robot_pose[0] = self.home_hrc[0] + self.k * corrected_target_pose[0]
		self.robot_pose[1] = self.home_hrc[1] - self.k * corrected_target_pose[1]
		self.robot_pose[2] = self.home_hrc[2] + self.k * corrected_target_pose[2]
		self.robot_pose[3:] = self.home_hrc[3:]

		if(self.right_hand_pose.orientation.w > 0.707 and self.right_hand_pose.orientation.x < 0.707):
			self.status = 'HRC/idle'
		
		elif(self.hand_grip_strength.data > 75):
			left, right = self.call_hand_calib_server()
			self.robot_colift_init = self.rtde_r.getActualTCPPose()
			self.status = 'HRC/colift'
			
		self.motion_hand_colift_init = self.left_hand_pose

	def hrc_colift(self):
		# TODO: any problem coming from IDLE but not from APPROACH?
		''' Make force thingy here '''
		vector = [0.0, 1.0, 0.0] # A pose vector that defines the force frame relative to the base frame.
		selection_vector = [0.0, 1.0, 0.0, 0.0, 1.0, 0.0] # A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame
		wrench = [0, 5, 0] # The forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque. Values have no effect for non-compliant axes
		type = 1 # An integer [1;3] specifying how the robot interprets the force frame. 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing from the robot tcp towards the origin of the force frame. 2: The force frame is not transformed. 3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp velocity vector onto the x-y plane of the force frame.
		limits = [1500, 1500, 1500, 0, 0, 0]# (Float) 6d vector. For compliant axes, these values are the maximum allowed tcp speed along/about the axis. For non-compliant axes, these values are the maximum allowed deviation along/about an axis between the actual tcp position and the one set by the program.
		self.rtde_c.forceMode(vector, selection_vector, wrench, type)
		lift_axis = 0

		# TODO: fix this. won't work like that. Need proper data type
		_zero_acc = [0.0, 0.0, 0.0]
		_result = np.isclose(self.rtde_r.getActualToolAccelerometer(), _zero_acc)
		if _result[lift_axis]:
			# while in HRC/colift
			while self.status == 'HRC/colift':
				# get desired axis
				left, right = self.call_hand_calib_server()
				# TODO: check if x is the correct axis for the side motion
				# TODO: check if 0.4 TH is enough or too much
				if(self.left_hand_pose.position.x > 0.4):
					vector = [1.0, 0.0, 0.0]
				elif(self.left_hand_pose.position.x < -0.4):
					vector = [-1.0, 0.0, 0.0]
				else:
					vector = [-1.0, 0.0, 0.0] # complient in all axes
				# set force to that axis
				self.rtde_c.forceMode(vector, selection_vector, wrench, type)
				# check if still in colift
				if(self.right_hand_pose.position.x < -0.25 and self.right_hand_pose.position.z < -0.15):
					self.rtde_c.forceModeStop()
					self.status = 'HRC/release'
				elif(self.right_hand_pose.orientation.w > 0.707 and self.right_hand_pose.orientation.x < 0.707):
					self.status = 'HRC/idle'	
					self.hrc_idle(from_colift=True)
				else:
					self.status = 'HRC/colift'
		# bool = self.rtde_c.isSteady()
		# std::vector<double> = self.rtde_r.getActualToolAccelerometer()

	def hrc_release(self):
		print("Moving to RELEASE pose")
		self.rtde_c.servoStop()
		self.rtde_c.moveJ_IK(self.release_before)
		self.rtde_c.moveJ(self.release)
		cmd_release = Bool()
		cmd_release = False
		self.pub_grip_cmd.publish(cmd_release)
		print("Robot at RELEASE")
		rospy.sleep(4)  # Wait until the gripper is fully open
		# user_input = raw_input("Move to RELEASE APPROACH pose?")
		# if user_input == 'y':
		self.rtde_c.moveJ_IK(self.release_after)
		print("Robot at RELEASE APPROACH")

		self.status = 'HRC/release'



	def update2(self, x):
		try:
			# self.rtde_c.servoL([self.robot_init[0], self.robot_init[1], self.robot_init[2], self.robot_init[3]+self.tcp_ori.x, self.robot_init[4]+self.tcp_ori.y, self.robot_init[5]+self.tcp_ori.z], 0.5, 0.3, 0.002, 0.1, 300)

			# self.right_arm_move_left_arm_ori()
			# print(self.robot_init[0]-self.robot_pose[0],
			# 	  self.robot_init[1]-self.robot_pose[1],
			# 	  self.robot_init[2]-self.robot_pose[2])
			# self.rtde_c.servoL([self.robot_pose[0], self.robot_pose[1], self.robot_pose[2], self.robot_init[3], self.robot_init[4], self.robot_init[5]+x], 0.5, 0.3, 0.002, 0.1, 300)

			return self.status
		except KeyboardInterrupt:
			self.rtde_c.stopScript()
			raise


	def update(self):
		# State machine here
		pass

		# After the state machine
		user_input = input("Ready to new cycle?")
		if user_input == 'y':
			self.rtde_c.moveJ_IK(self.home_teleop_approach)
			self.rtde_c.moveJ(self.home_teleop)
			self.role = "HUMAN_LEADING"
			self.state = "IDLE"
			self.status = 'teleop/idle'
	
		# print("state:", self.state, "    role:", self.role)
		print("status:", self.status)
		# self.hrc_status = self.state + ',' + self.role
		self.hrc_status = self.status
		self.pub_hrc_status.publish(self.hrc_status)
		self.robot_current_TCP.data = self.rtde_r.getActualTCPPose()
		self.pub_robot_current_TCP.publish(self.robot_current_TCP)

#! /usr/bin/env python3

"""
Subscribes two hand poses and drives the real UR5e robot in real-time.
"""
import sys
import rospy
from math import pi
from math import radians as d2r
import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int16, Float64, Bool
from std_msgs.msg import Float32MultiArray

from . import Kinematics_with_Quaternions as kinematic

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

elbow_height_th = 0.2


class RobotCommander:
	def __init__(self, rate=100, start_node=False, s=1.0, k=1.0):
		"""Initializes the robot commander
			@params s: motion hand - steering hand scale
			@params k: target hand pose - robot pose scale"""
		self.rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
		self.rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")

		if start_node == True:
			rospy.init_node("robot_move_with_ur_rtde")
			self.r = rospy.Rate(rate)
			print("robot_move_with_ur_rtde Node Created")


		self.robot_init = self.rtde_r.getActualTCPPose()
		self.robot_current_TCP = Float32MultiArray()
		self.robot_init_joints = self.rtde_r.getActualQ()
		# self.release_prev_joints = [d2r(-156.66), d2r(-40.38), d2r(58.48), d2r(-16.06), d2r(38.70), d2r(-94.83)]
		# self.release_approach_joints = [d2r(-175.82), d2r(-44.7), d2r(85.04), d2r(-36.80), d2r(19.64), d2r(-96.58)]
		# self.release_joints = [d2r(-156.66), d2r(-33.67), d2r(62.73), d2r(-27.07), d2r(38.72), d2r(-94.81)]
		# self.home_approach_joints = [d2r(-175.82), d2r(-58.56), d2r(73.26), d2r(-11.08), d2r(19.57), d2r(-96.67)]
		self.release_prev_joints = [d2r(-149.60), d2r(-58.63), d2r(59.90), d2r(7.99), d2r(41.82), d2r(-97.12)]
		self.release_approach_joints = [d2r(-158.48), d2r(-61.75), d2r(87.86), d2r(-14.92), d2r(33.14), d2r(-99.55)]
		self.release_joints = [d2r(-147.37), d2r(-51.78), d2r(71.37), d2r(-10.75), d2r(44.05), d2r(-96.53)]
		self.home_approach_joints = [d2r(-95.72), d2r(-85.83), d2r(95.93), d2r(-3.86), d2r(95.29), d2r(-89.49)]

		print("============ Arm current pose: ", self.robot_init)
		# self.rtde_c.moveJ(self.release_joints)
		# print("click Enter to continue")
		# dummy_input = input()
		# sys.exit()

		self.home = self.robot_init
		self.target_pose = Pose()
		self.motion_hand_pose = Pose()
		self.hand_grip_strength = Int16()
		self.steering_hand_pose = Pose()
		self.robot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.merged_hands_pose = Pose()

		self.robot_colift_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.target_pose_colift_init = Pose()
		self.motion_hand_colift_init = Pose()
		self.motion_hand_colift_pos_ch = Point()

		self.hand_init_orientation = Quaternion()
		self.human_to_robot_init_orientation = Quaternion(0.0, 0.0, 0.707, 0.707)
		# This human-robot init orientation should not be static. TODO: make it based on chest orientation.

		self.s = s
		self.k = k

		self.init_flag = False
		self.colift_flag = False
		self.joint_flag = False

		self.state = "IDLE"
		self.role = "HUMAN_LEADING"  # or "ROBOT_LEADING"
		self.hrc_status = String()


		self.elbow_left_height = 0.0
		self.elbow_right_height = 0.0
		self.force_human = Float64()
		self.force_mode = String()

		self.colift_dir = 'up'
		self.colift_flag = False
		self.hrc_hand_calib_flag = False
		self.hrc_colift_calib_flag = False
		self.wrist_calib_flag = False
		self.tcp_ori = Vector3()
		self.tcp_ori_init = Vector3()

		self.do_flag = 0
               

	def init_subscribers_and_publishers(self):
		global elbow_height_th
		self.sub_hand_grip_strength = rospy.Subscriber('/robotiq_grip_gap', Int16, self.cb_hand_grip_strength)
		self.sub_motion_hand_pose = rospy.Subscriber('/motion_hand_pose', Pose, self.cb_motion_hand_pose)
		self.sub_steering_pose = rospy.Subscriber('/steering_hand_pose', Pose, self.cb_steering_pose)
		self.sub_human_ori = rospy.Subscriber('/human_ori', Quaternion, self.cb_human_ori)
		self.pub_tee_goal = rospy.Publisher('/Tee_goal_pose', Pose, queue_size=1)
		self.pub_hrc_status = rospy.Publisher('/hrc_status', String, queue_size=1)
		self.pub_grip_cmd = rospy.Publisher('/cmd_grip_bool', Bool, queue_size=1)

		self.pub_merged_hands = rospy.Publisher('/merged_hands', Pose, queue_size=1)

		self.pub_force_human = rospy.Publisher('/human_force', Float64, queue_size=1)
		self.pub_force_mode = rospy.Publisher('/force_mode', String, queue_size=1)

		self.pub_robot_current_TCP = rospy.Publisher('/robot_current_TCP', Float32MultiArray, queue_size=10)
		self.pub_robot_current_TCP_pose = rospy.Publisher('/robot_current_TCP_pose', Pose, queue_size=10)
		self.sub_elbow_left = rospy.Subscriber('/elbow_left', Pose, self.cb_elbow_left)
		self.sub_elbow_right= rospy.Subscriber('/elbow_right', Pose, self.cb_elbow_right)

		try:
			elbow_height_th = rospy.get_param("/elbow_height_th")
			print("elbow_height_th parameter set:", elbow_height_th)
		except:
			print("no elbow height parameter set")
		

	
	def cb_elbow_left(self, msg):
		self.elbow_left_height = msg.position.y

		
	def cb_elbow_right(self, msg):
		self.elbow_right_height = -msg.position.y


	def cb_human_ori(self, msg):
		""" Subscribes chest IMU orientation to map human w.r.t the world frame """
		self.human_to_robot_init_orientation = kinematic.q_multiply(Quaternion(0.0, 0.0, 0.707, 0.707), msg)


	def cb_hand_grip_strength(self, msg):
		""" Subscribes hand grip strength
		Open: 0
		Close: 255 """
		self.hand_grip_strength = msg


	def cb_motion_hand_pose(self, msg):
		""" Subscribes left hand pose """
		self.motion_hand_pose = msg
		if not self.init_flag:
			self.hand_init_orientation = kinematic.q_invert(self.steering_hand_pose.orientation)
			print("Hand init set:", self.hand_init_orientation)
			self.init_flag = True
	

	def cb_steering_pose(self, msg):
		""" Subscribes right hand pose """
		self.steering_hand_pose = msg


	def cartesian_control_2_arms(self):	
		self.target_pose.position.x = (- self.motion_hand_pose.position.x) - self.s * self.steering_hand_pose.position.x
		self.target_pose.position.y = (- self.motion_hand_pose.position.y) - self.s * self.steering_hand_pose.position.y
		self.target_pose.position.z = self.motion_hand_pose.position.z + self.s * self.steering_hand_pose.position.z
		self.target_pose.orientation = self.motion_hand_pose.orientation

		# print "robot_pose:", self.robot_pose.position
		corrected_target_pose = kinematic.q_rotate(self.human_to_robot_init_orientation, self.target_pose.position)
		self.robot_pose[0] = self.robot_init[0] + self.k * corrected_target_pose[0]
		self.robot_pose[1] = self.robot_init[1] - self.k * corrected_target_pose[1]
		self.robot_pose[2] = self.robot_init[2] + self.k * corrected_target_pose[2]
		# self.robot_pose.orientation = kinematic.q_multiply(self.robot_init.orientation, kinematic.q_multiply(self.hand_init_orientation, self.motion_hand_pose.orientation))
		self.robot_pose[3:] = self.robot_init[3:]

		self.merged_hands_pose.position.x = self.k * corrected_target_pose[0]
		self.merged_hands_pose.position.y = - self.k * corrected_target_pose[1]
		self.merged_hands_pose.position.z = self.k * corrected_target_pose[2]
		print(self.merged_hands_pose.position)
		self.pub_merged_hands.publish(self.merged_hands_pose)

		self.motion_hand_colift_init = self.motion_hand_pose


	def cartesian_control_1_arm(self):	
		''' Make force thingy here '''
		vector = self.rtde_r.getActualTCPPose() # A pose vector that defines the force frame relative to the base frame.
		selection_vector = [0, 0, 0, 0, 0, 0] # A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame
		wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # The forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque. Values have no effect for non-compliant axes
		type = 2 # An integer [1;3] specifying how the robot interprets the force frame. 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing from the robot tcp towards the origin of the force frame. 2: The force frame is not transformed. 3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp velocity vector onto the x-y plane of the force frame.
		limits = [0.1, 0.1, 0.1, 0.17, 0.17, 0.17]# (Float) 6d vector. For compliant axes, these values are the maximum allowed tcp speed along/about the axis. For non-compliant axes, these values are the maximum allowed deviation along/about an axis between the actual tcp position and the one set by the program.

		_curr_force = self.rtde_r.getActualTCPForce()
		# print(_curr_force[0])

		if self.colift_dir == 'right':
			print(self.state)
			vector = self.rtde_r.getActualTCPPose()
			selection_vector = [0, 1, 0, 0, 0, 0] 
			wrench = [0.0, 30.0, 0.0, 0.0, 0.0, 0.0]
			limits = [0.1, 0.5, 0.1, 0.17, 0.17, 0.17]

		elif self.colift_dir == 'left':
			print(self.state)
			vector = self.rtde_r.getActualTCPPose()
			selection_vector = [0, 1, 0, 0, 0, 0]
			wrench = [0.0, -30.0, 0.0, 0.0, 0.0, 0.0]
			limits = [0.1, 0.5, 0.1, 0.17, 0.17, 0.17]

		elif self.colift_dir == 'up':
			print(self.state)
			vector = self.rtde_r.getActualTCPPose()
			selection_vector = [1, 0, 0, 0, 0, 0]
			wrench = [-40.0, 0.0, 0.0, 0.0, 0.0, 0.0]
			limits = [0.5, 0.1, 0.1, 0.17, 0.17, 0.17]
		
		print(_curr_force[1])
		
		if abs(_curr_force[1]) > 27.0:
		# if self.tcp_ori.x > 0.6:
			print("Side movement")
			elbow_height_th = 0.2
			# colift_dir = ''
			# colift_dir_past = ''
			if((self.elbow_right_height > elbow_height_th) and (self.elbow_left_height < elbow_height_th)):
				self.colift_dir = 'right'
			elif((self.elbow_left_height > elbow_height_th) and (self.elbow_right_height < elbow_height_th)):
				self.colift_dir = 'left'
			elif((self.elbow_left_height > elbow_height_th) and (self.elbow_right_height > elbow_height_th)):
				self.colift_dir = 'up'
			else:
				if not self.colift_flag:
					self.colift_flag = True
				else:
					self.colift_dir = 'null'
				
		# check if still in colift
		if(self.steering_hand_pose.position.x < -0.25 and self.steering_hand_pose.position.z < -0.15):
			self.rtde_c.servoStop()
			self.rtde_c.forceModeStop()
			self.state = "RELEASE"
			print('HRC/release')
			self.do_flag = 0
		elif(self.steering_hand_pose.orientation.w > 0.707 and self.steering_hand_pose.orientation.x < 0.707):
			# self.rtde_c.forceModeStop()
			self.state = "IDLE"
			print('HRC/idle')
			# self.hrc_idle(from_colift=True)
		elif(self.hand_grip_strength.data < 100):
			self.state = "APPROACH"
		else:
			self.state = "CO-LIFT"
		# self.robot_pose = self.rtde_c.getTargetWaypoint()
		# bool = self.rtde_c.isSteady()
		# std::vector<double> = self.rtde_r.getActualToolAccelerometer()
		print(self.colift_dir)
		print("self.elbow_left_height, self.elbow_right_height")
		print(self.elbow_left_height, self.elbow_right_height)
		self.force_human.data = _curr_force[1]
		self.force_mode.data = self.colift_dir
		self.pub_force_human.publish(self.force_human)
		self.pub_force_mode.publish(self.force_mode)
		self.rtde_c.forceMode(vector, selection_vector, wrench, type, limits)


	def update(self):
		global robot_colift_init
		# Palm up: active, palm dowm: idle
		if not self.role == "ROBOT_LEADING":
			if(self.state == "CO-LIFT"):
				# print("self.steering_hand_pose.position.x", self.steering_hand_pose.position.x)
				# print(" self.steering_hand_pose.position.z",  self.steering_hand_pose.position.z)
				if(self.steering_hand_pose.position.x < -0.25 and self.steering_hand_pose.position.z < -0.15):
					self.rtde_c.servoStop()
					self.rtde_c.forceModeStop()
					self.state = "RELEASE"
				else:
					if(self.steering_hand_pose.orientation.w > 0.707 and self.steering_hand_pose.orientation.x < 0.707):
						self.state = "IDLE"
					else:
						self.state == "CO-LIFT"
						if not self.colift_flag:
							self.robot_colift_init = self.rtde_r.getActualTCPPose()
							self.colift_flag = True
			elif(self.steering_hand_pose.orientation.w > 0.707 and self.steering_hand_pose.orientation.x < 0.707):
				# print(self.steering_hand_pose.orientation.w, self.steering_hand_pose.orientation.x)
				self.state = "IDLE"
				# if steering arm vertically downwords when it is in IDLE
				# if(self.steering_hand_pose.position.x < -0.3 and self.steering_hand_pose.position.z < -0.4):
				# 	self.state = "RELEASE"
			elif(self.steering_hand_pose.orientation.w < 0.707 and self.steering_hand_pose.orientation.x > 0.707):
				if not (self.state == "CO-LIFT"):
					self.state = "APPROACH"

			try:
				if(self.state == "APPROACH" or self.state == "CO-LIFT"): # ACTIVE
					# check grip here
					# print "motion:", self.motion_hand_pose.position, "hands:", self.target_pose.position
					# print("self.hand_grip_strength.data:", self.hand_grip_strength.data)
					if(self.hand_grip_strength.data > 100):
						prev_state = self.state
						self.state = "CO-LIFT"
						if prev_state != "CO-LIFT":
							print("here")
							self.rtde_c.servoStop()
						self.cartesian_control_1_arm()  # one hand free
						# do something extra? Change axes? Maybe robot take over from here?
						# No way to leave CO-LIFT state unless hand releases
					else:
						self.cartesian_control_2_arms()
						
					robot_pose_pose = Pose(Point(self.robot_pose[0], self.robot_pose[1], self.robot_pose[2]), Quaternion())
					self.pub_tee_goal.publish(robot_pose_pose)
					self.rtde_c.servoL(self.robot_pose, 0.5, 0.3, 0.002, 0.1, 300)
				
				elif(self.state == "IDLE"):
					pass
				elif(self.state == "RELEASE"):
					self.role = "ROBOT_LEADING"
				else:
					raise AssertionError("Unknown collaboration state")

			except AssertionError as e:
				print(e)

		else:

			print("state:", self.state, "    role:", self.role)
			self.hrc_status = self.state + ',' + self.role
			self.pub_hrc_status.publish(self.hrc_status)
			self.robot_current_TCP.data = self.rtde_r.getActualTCPPose()
			self.pub_robot_current_TCP.publish(self.robot_current_TCP)
			robot_current_TCP_pose = kinematic.list_to_pose(self.robot_current_TCP.data)
			self.pub_robot_current_TCP_pose.publish(robot_current_TCP_pose)
			
			## RELEASE (or PLACE)
			print("Moving to RELEASE pose")
			self.rtde_c.servoStop()
			self.rtde_c.forceModeStop()
			pose_goal_list = self.rtde_c.getForwardKinematics(self.home_approach_joints)
			pose_goal = kinematic.list_to_pose(pose_goal_list)
			pose_goal.position.x = -pose_goal.position.x
			pose_goal.position.y = -pose_goal.position.y
			pose_goal.position.z = -pose_goal.position.z
			self.pub_tee_goal.publish(pose_goal)
			self.rtde_c.moveJ(self.home_approach_joints)

			print("state:", self.state, "    role:", self.role)
			self.hrc_status = self.state + ',' + self.role
			self.pub_hrc_status.publish(self.hrc_status)
			self.robot_current_TCP.data = self.rtde_r.getActualTCPPose()
			self.pub_robot_current_TCP.publish(self.robot_current_TCP)
			robot_current_TCP_pose = kinematic.list_to_pose(self.robot_current_TCP.data)
			self.pub_robot_current_TCP_pose.publish(robot_current_TCP_pose)

			pose_goal_list = self.rtde_c.getForwardKinematics(self.release_prev_joints)
			pose_goal = kinematic.list_to_pose(pose_goal_list)
			pose_goal.position.x = -pose_goal.position.x
			pose_goal.position.y = -pose_goal.position.y
			pose_goal.position.z = -pose_goal.position.z
			self.pub_tee_goal.publish(pose_goal)
			self.rtde_c.moveJ(self.release_prev_joints)

			print("state:", self.state, "    role:", self.role)
			self.hrc_status = self.state + ',' + self.role
			self.pub_hrc_status.publish(self.hrc_status)
			self.robot_current_TCP.data = self.rtde_r.getActualTCPPose()
			self.pub_robot_current_TCP.publish(self.robot_current_TCP)
			robot_current_TCP_pose = kinematic.list_to_pose(self.robot_current_TCP.data)
			self.pub_robot_current_TCP_pose.publish(robot_current_TCP_pose)

			pose_goal_list = self.rtde_c.getForwardKinematics(self.release_joints)
			pose_goal = kinematic.list_to_pose(pose_goal_list)
			pose_goal.position.x = -pose_goal.position.x
			pose_goal.position.y = -pose_goal.position.y
			pose_goal.position.z = -pose_goal.position.z
			self.pub_tee_goal.publish(pose_goal)
			self.rtde_c.moveJ(self.release_joints)
			cmd_release = Bool()
			cmd_release = False
			self.pub_grip_cmd.publish(cmd_release)
			print("Robot at RELEASE")

			print("state:", self.state, "    role:", self.role)
			self.hrc_status = self.state + ',' + self.role
			self.pub_hrc_status.publish(self.hrc_status)
			self.robot_current_TCP.data = self.rtde_r.getActualTCPPose()
			self.pub_robot_current_TCP.publish(self.robot_current_TCP)
			robot_current_TCP_pose = kinematic.list_to_pose(self.robot_current_TCP.data)
			self.pub_robot_current_TCP_pose.publish(robot_current_TCP_pose)


			# ## RELEASE APPROACH
			rospy.sleep(4)  # Wait until the gripper is fully open
			pose_goal_list = self.rtde_c.getForwardKinematics(self.release_approach_joints)
			pose_goal = kinematic.list_to_pose(pose_goal_list)
			pose_goal.position.x = -pose_goal.position.x
			pose_goal.position.y = -pose_goal.position.y
			pose_goal.position.z = -pose_goal.position.z
			self.pub_tee_goal.publish(pose_goal)
			self.rtde_c.moveJ(self.release_approach_joints)
			print("Robot at release approach")
			
			print("state:", self.state, "    role:", self.role)
			self.hrc_status = self.state + ',' + self.role
			self.pub_hrc_status.publish(self.hrc_status)
			self.robot_current_TCP.data = self.rtde_r.getActualTCPPose()
			self.pub_robot_current_TCP.publish(self.robot_current_TCP)
			robot_current_TCP_pose = kinematic.list_to_pose(self.robot_current_TCP.data)
			self.pub_robot_current_TCP_pose.publish(robot_current_TCP_pose)

			## GO BACK HOME
			print("Please move arms such that role:HUMAN_LEADING and state:IDLE")
			user_input = input("Ready to new cycle?")
			if user_input == 'y':
				pose_goal_list = self.rtde_c.getForwardKinematics(self.home_approach_joints)
				pose_goal = kinematic.list_to_pose(pose_goal_list)
				pose_goal.position.x = -pose_goal.position.x
				pose_goal.position.y = -pose_goal.position.y
				pose_goal.position.z = -pose_goal.position.z
				self.pub_tee_goal.publish(pose_goal)
				self.rtde_c.moveJ(self.home_approach_joints)

				print("state:", self.state, "    role:", self.role)
				self.hrc_status = self.state + ',' + self.role
				self.pub_hrc_status.publish(self.hrc_status)
				self.robot_current_TCP.data = self.rtde_r.getActualTCPPose()
				self.pub_robot_current_TCP.publish(self.robot_current_TCP)
				robot_current_TCP_pose = kinematic.list_to_pose(self.robot_current_TCP.data)
				self.pub_robot_current_TCP_pose.publish(robot_current_TCP_pose)

				pose_goal_list = self.rtde_c.getForwardKinematics(self.robot_init_joints)
				pose_goal = kinematic.list_to_pose(pose_goal_list)
				pose_goal.position.x = -pose_goal.position.x
				pose_goal.position.y = -pose_goal.position.y
				pose_goal.position.z = -pose_goal.position.z
				self.pub_tee_goal.publish(pose_goal)
				self.rtde_c.moveJ(self.robot_init_joints)

				print("state:", self.state, "    role:", self.role)
				self.hrc_status = self.state + ',' + self.role
				self.pub_hrc_status.publish(self.hrc_status)
				self.robot_current_TCP.data = self.rtde_r.getActualTCPPose()
				self.pub_robot_current_TCP.publish(self.robot_current_TCP)
				robot_current_TCP_pose = kinematic.list_to_pose(self.robot_current_TCP.data)
				self.pub_robot_current_TCP_pose.publish(robot_current_TCP_pose)

				# rospy.sleep(5)
				self.role = "HUMAN_LEADING"
				self.state = "IDLE"
		
		print("state:", self.state, "    role:", self.role)
		self.hrc_status = self.state + ',' + self.role
		self.pub_hrc_status.publish(self.hrc_status)
		self.robot_current_TCP.data = self.rtde_r.getActualTCPPose()
		self.pub_robot_current_TCP.publish(self.robot_current_TCP)
		robot_current_TCP_pose = kinematic.list_to_pose(self.robot_current_TCP.data)
		self.pub_robot_current_TCP_pose.publish(robot_current_TCP_pose)
		self.r.sleep()

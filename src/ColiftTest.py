#!/usr/bin/env python3

# imports
import rospy
import time
import numpy
from math import radians as d2r
from math import degrees as r2d
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Quaternion, PoseStamped, Point, Pose
from std_msgs.msg import Float64, Int16

import sys
sys.path.insert(0, "/home/gizem/catkin_ws/src/my_human_pkg/src")
import Classes.Kinematics_with_Quaternions_py3 as kinematichuman_joint_imu
from Classes.IMU_class_single import IMUsubscriber

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

ur_cmd = Vector3()
ur_cmd_init = Vector3()
diff_time = Float64()
prev = time.time()

diff_time_ur = Float64()
prev_ur = time.time()

calib_flag = False

e1 = 0
e2 = 0

def cb_test_move(msg):
    pass

def cb_e1(msg):
	global e1
	e1 = msg.data
	print(e1)

def cb_e2(msg):
	global e2
	e2 = msg.data


def cb_elbow_left(self, msg):
	global e1
	e1 = msg.position.y

	
def cb_elbow_right(self, msg):
	global e2
	e2 = -msg.position.y

    


def main():
	global e1, e2
	rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
	rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")
	rospy.init_node('ur_rtde_test')
	sub = rospy.Subscriber('/sensor_r_elbow_rpy', Vector3, cb_test_move)
	sub = rospy.Subscriber('/e1', Int16, cb_e1)
	sub = rospy.Subscriber('/e2', Int16, cb_e2)
	pub = rospy.Publisher('/ur_cmd', Vector3, queue_size=1)

	sub_elbow_left = rospy.Subscriber('/elbow_left', Pose, cb_elbow_left)
	sub_elbow_right= rospy.Subscriber('/elbow_right', Pose, cb_elbow_right)

	rate = rospy.Rate(125)

	# rtde_c.moveL([0.106, 0.715, -0.0, 2.616, -2.477, -1.510], 0.5, 0.3)
	t = rtde_r.getActualTCPPose()
	# rtde_c.moveL([t[0], t[1], t[2], t[3], t[4]+0.2, t[5]], 0.5, 0.3)
	print(t)

	dt = 1.0/500  # 2ms
	lookahead_time = 0.1
	gain = 300

	colift_dir = "up"
	colift_flag = False


	while not rospy.is_shutdown():
		vector = rtde_r.getActualTCPPose() # A pose vector that defines the force frame relative to the base frame.
		selection_vector = [0, 0, 0, 0, 0, 0] # A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame
		wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # The forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque. Values have no effect for non-compliant axes
		type = 2 # An integer [1;3] specifying how the robot interprets the force frame. 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing from the robot tcp towards the origin of the force frame. 2: The force frame is not transformed. 3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp velocity vector onto the x-y plane of the force frame.
		limits = [0.1, 0.1, 0.1, 0.17, 0.17, 0.17]# (Float) 6d vector. For compliant axes, these values are the maximum allowed tcp speed along/about the axis. For non-compliant axes, these values are the maximum allowed deviation along/about an axis between the actual tcp position and the one set by the program.

		_curr_force = rtde_r.getActualTCPForce()
		# print(_curr_force[0])

		if colift_dir == 'right':
			vector = rtde_r.getActualTCPPose()
			selection_vector = [0, 1, 0, 0, 0, 0] 
			wrench = [0.0, 20.0, 0.0, 0.0, 0.0, 0.0]
			limits = [0.1, 0.5, 0.1, 0.17, 0.17, 0.17]

		elif colift_dir == 'left':
			vector = rtde_r.getActualTCPPose()
			selection_vector = [0, 1, 0, 0, 0, 0]
			wrench = [0.0, -20.0, 0.0, 0.0, 0.0, 0.0]
			limits = [0.1, 0.5, 0.1, 0.17, 0.17, 0.17]

		elif colift_dir == 'up':
			vector = rtde_r.getActualTCPPose()
			selection_vector = [1, 0, 0, 0, 0, 0]
			wrench = [-30.0, 0.0, 0.0, 0.0, 0.0, 0.0]
			limits = [0.5, 0.1, 0.1, 0.17, 0.17, 0.17]
		
		print(_curr_force)
		print(colift_dir)
		if abs(_curr_force[1]) > 27.0:
			# if tcp_ori.x > 0.6:
			# print("Side movement")
			# height_th = 100
			height_th = 0.15
			# colift_dir = ''
			# colift_dir_past = ''
			if((e1 > height_th) and (e2 < height_th)):
				colift_dir = 'right'
			elif((e1 < height_th) and (e2 > height_th)):
				colift_dir = 'left'
			elif((e1 > height_th) and (e2 > height_th)):
				colift_dir = 'up'
			else:
				if not colift_flag:
					colift_flag = True
				else:
					colift_dir = 'null'	
			print(colift_dir)

		rtde_c.forceMode(vector, selection_vector, wrench, type, limits)
		rate.sleep()


if __name__ == '__main__':
    calib_flag = False
    main()
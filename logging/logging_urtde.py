#!/usr/bin/env python3

"""This is the script that works as a ROS node.
		TODO: make it as gui as you did before.
"""

import re
import rospy, time
import Data.data_logger_urtde as data_logger
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Imu

# from get_model_gazebo_pose import GazeboModel

lhand_pose = Pose()
rhand_pose = Pose()
hand_pose = Pose()
tgoal_pose = Pose()
tactual_pose = Pose()
status = String()

force_mode = String()
force_human = Float64()
lelbow = 0
relbow = 0
current_tcp_pose = Pose()

table_imu = Imu()
table_acc = Vector3()
table_angle = Vector3()


def callback_lhand_pose(msg):
	global lhand_pose
	lhand_pose = msg	

def callback_rhand_pose(msg):
	global rhand_pose
	rhand_pose = msg	

def callback_hand_pose(msg):
	global hand_pose
	hand_pose = msg	

def callback_tool_goal_pose(msg):
	global tgoal_pose
	tgoal_pose = msg	

def callback_tool_actual_pose(msg):
	global tactual_pose
	# TODO: wrist_3_link to tool tf here if in gazebo
	# or run ur5_with_hang_gazebo/src/tool0_listener
	## rosrun ur5_with_hand_gazebo tool0_listener.py
	## UPDATE: rosrun arm_motion_controller_pkg baselink_to_tool0_tf
	tactual_pose = msg


def callback_hrc_status(msg):
	global status
	status = msg.data	


def callback_force_mode(msg):
	global force_mode
	force_mode = msg	

def callback_human_force(msg):
	global force_human
	force_human = msg

def callback_lelbow(msg):
	global lelbow
	lelbow = msg.position.y

def callback_relbow(msg):
	global relbow
	relbow = -msg.position.y

def callback_tcp_pose(msg):
	global current_tcp_pose
	current_tcp_pose = msg

def callback_table_imu(msg):
    global table_acc,table_imu
    table_acc = msg.linear_acceleration

def callback_table_angle(msg):
    global table_angle
    table_angle = msg


	
if __name__ == "__main__":
	try:
		rospy.init_node('data_logger_node')
		start_time = time.time()
		current_time = rospy.get_time()
		data_logger.enable_logging()

		sub_lhand_pose = rospy.Subscriber('/wrist_left', Pose, callback_lhand_pose)
		sub_rhand_pose = rospy.Subscriber('/wrist_right', Pose, callback_rhand_pose)
		sub_hand_pose = rospy.Subscriber('/merged_hands', Pose, callback_hand_pose)
		sub_tool_goal_pose = rospy.Subscriber('/Tee_goal_pose', Pose, callback_tool_goal_pose)
		sub_tool_actual_pose = rospy.Subscriber('/base_to_tool', Pose, callback_tool_actual_pose) # /base_link to /tool0 TF
		sub_hrc_status = rospy.Subscriber('/hrc_status', String, callback_hrc_status) 
		sub_force_mode = rospy.Subscriber('/force_mode', String, callback_force_mode)
		sub_human_force = rospy.Subscriber('/human_force', Float64, callback_human_force)

		sub_elbow_left = rospy.Subscriber('/elbow_left', Pose, callback_lelbow)
		sub_elbow_right= rospy.Subscriber('/elbow_right', Pose, callback_relbow)

		sub_current_tcp = rospy.Subscriber('/robot_current_TCP_pose', Pose, callback_tcp_pose)

		sub_table_acc = rospy.Subscriber('sensor_l_wrist', Imu, callback_table_imu) 
		sub_table_angle = rospy.Subscriber('sensor_l_wrist_rpy', Vector3, callback_table_angle) 

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			elapsed_time = time.time() - start_time
			data_logger.log_metrics(elapsed_time, lhand_pose, rhand_pose, hand_pose, tgoal_pose, tactual_pose, status, force_mode, force_human, lelbow, relbow, current_tcp_pose, table_acc, table_angle)
			rate.sleep()
	except KeyboardInterrupt:
		data_logger.disable_logging()
		rospy.signal_shutdown("KeyboardInterrupt")
		raise
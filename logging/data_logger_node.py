#!/usr/bin/env python3

"""This is the script that works as a ROS node.
		TODO: make it as gui as you did before.
"""

import rospy, time, sys
import Data.data_logger_class as data_logger
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

# from get_model_gazebo_pose import GazeboModel

lhand_pose = Pose()
rhand_pose = Pose()
hand_pose = Pose()
tgoal_pose = Pose()
tactual_pose = Pose()
tactual_corrected_pose = Pose()
status = String()


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


def callback_tool_corrected_pose(msg):
	global tactual_corrected_pose
	# TODO: wrist_3_link to tool tf here if in gazebo
	# or run ur5_with_hang_gazebo/src/tool0_listener
	## rosrun ur5_with_hand_gazebo tool0_listener.py
	## UPDATE: rosrun arm_motion_controller_pkg baselink_to_tool0_tf
	tactual_corrected_pose = msg

def callback_hrc_status(msg):
	global status
	status = msg	

	

if __name__ == "__main__":
	# global lhand_pose, rhand_pose, hand_pose, tgoal_pose, tactual_pose, tactual_corrected_pose
	try:
		rospy.init_node('data_logger_node')
		start_time = time.time()
		current_time = rospy.get_time()
		if not rospy.has_param('ref_frame'):
			print("Reference frame parameter has not set. Exiting...")
			sys.exit()
		else:
			param = rospy.get_param('ref_frame')
			print('here')
		username = input("Please enter username: ")
		ref_frame = input("Please enter reference frame: ")
		data_logger.enable_logging(username,ref_frame)

		sub_lhand_pose = rospy.Subscriber('/wrist_left', Pose, callback_lhand_pose)
		sub_rhand_pose = rospy.Subscriber('/wrist_right', Pose, callback_rhand_pose)
		sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, callback_hand_pose)
		sub_tool_goal_pose = rospy.Subscriber('/Tee_goal_pose', Pose, callback_tool_goal_pose)
		# sub_tool_actual_pose = rospy.Subscriber('/tool0_corrected', Pose, callback_tool_actual_pose) # /world to /tool0 TF
		sub_tool_actual_pose = rospy.Subscriber('/base_to_tool', Pose, callback_tool_actual_pose) # /base_link to /tool0 TF
		sub_tool_actual_pose = rospy.Subscriber('/base_to_tool_corrected', Pose, callback_tool_corrected_pose) # /base_link to /tool0 TF
		sub_hrc_status = rospy.Subscriber('/hrc_status', String, callback_hrc_status) 
		# or sub_tool_actual_pose = rospy.Subscriber('/Tee_calculated', Pose, callback_hand_pose) # /world to /tool0 TF
		# sub_tool_pose = rospy.Subscriber('/odom_wrist_3_link', Odometry, callback_tool_pose) ## Check this if it is the same as wrist_3_link.

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			elapsed_time = time.time() - start_time
			data_logger.log_metrics(elapsed_time, lhand_pose, rhand_pose, hand_pose, tgoal_pose, tactual_pose, tactual_corrected_pose, status)
			rate.sleep()
	except KeyboardInterrupt:
		data_logger.disable_logging()
		rospy.signal_shutdown("KeyboardInterrupt")
		raise
#! /usr/bin/env python3

"""
Subscrives two wrist poses w.r.t chest. Send Tee goal w.r.t UR5e initial pose at home.

UR5e init \base_link \tool0 TF at initial pose:
- Translation: [-0.136, 0.490, 0.687]
- Rotation: in Quaternion [-0.697, 0.005, 0.012, 0.717]
            in RPY (radian) [-1.542, 0.024, 0.010]
            in RPY (degree) [-88.374, 1.403, 0.549]
"""
import math, sys
import numpy as np

import rospy, actionlib
from arm_motion_controller_py3.msg import handCalibrationAction, handCalibrationFeedback, handCalibrationResult
from geometry_msgs.msg import Pose, Point

from Classes.DH_matrices import DHmatrices

DHmatrices = DHmatrices()

class HandCalibrate:
    def __init__(self):
        self.wrist_left_pose = Pose()
        self.wrist_right_pose = Pose()
        # self.wrist_left_pose.position = Point(1,2,3)
        # self.wrist_right_pose.position = Point(4,5,6)
        print("Initiated")

    def init_node(self, rate=100.0):
        self.pub_left_hand_pose = rospy.Publisher('/left_hand_pose', Pose, queue_size=1)
        self.pub_right_hand_pose = rospy.Publisher('/right_hand_pose', Pose, queue_size=1)
        self.sub_l_wrist = rospy.Subscriber('/wrist_left', Pose, self.cb_l_wrist)
        self.sub_r_wrist = rospy.Subscriber('/wrist_right', Pose, self.cb_r_wrist)

        rospy.init_node('wrist_to_robot_2arms')
        print("hands_calibrate node started")
        self.rate = rospy.Rate(rate)

        self.a_server = actionlib.SimpleActionServer(
                "hand_calibration_as", handCalibrationAction, execute_cb=self.as_execute_cb, auto_start=False)
        self.a_server.start()

        
        print("Move to initial arm poses in 4 seconds...")
        # rospy.sleep(1)
        # print("3 seconds...")
        # rospy.sleep(1)
        # print("2 seconds...")
        # rospy.sleep(1)
        # print("1 second...")
        # rospy.sleep(1)

        self.calc_inv()

    def calc_inv(self):
        self.left_htm_init = DHmatrices.pose_to_htm(self.wrist_left_pose)
        self.right_htm_init = DHmatrices.pose_to_htm(self.wrist_right_pose)
        print("Initial arm poses registered")

        self.left_htm_init_inv = np.linalg.inv(self.left_htm_init)
        self.right_htm_init_inv = np.linalg.inv(self.right_htm_init)

    def update(self):
        tf_left = np.matmul(self.left_htm_init_inv, DHmatrices.pose_to_htm(self.wrist_left_pose)) 
        tf_right = np.matmul(self.right_htm_init_inv, DHmatrices.pose_to_htm(self.wrist_right_pose))
        self.tf_left_pose = DHmatrices.htm_to_pose(tf_left)
        self.tf_right_pose = DHmatrices.htm_to_pose(tf_right)
        self.pub_left_hand_pose.publish(self.tf_left_pose)
        self.pub_right_hand_pose.publish(self.tf_right_pose)


    def cb_l_wrist(self, msg):
        self.wrist_left_pose = msg

    def cb_r_wrist(self, msg):
        self.wrist_right_pose = msg
    
    def as_execute_cb(self, goal):

        success = True
        calib_flag = True
        feedback = handCalibrationFeedback()
        result = handCalibrationResult()

        if goal.calib_request:
            self.calc_inv()
            feedback.calib_success = calib_flag
            result.left_hand_pose = [self.tf_left_pose.position.x, self.tf_left_pose.position.y, self.tf_left_pose.position.z]
            result.right_hand_pose = [self.tf_right_pose.position.x, self.tf_right_pose.position.y, self.tf_right_pose.position.z]
        else:
            feedback.calib_success = False # not calib_flag
        self.a_server.publish_feedback(feedback)

        if success:
            self.a_server.set_succeeded(result)

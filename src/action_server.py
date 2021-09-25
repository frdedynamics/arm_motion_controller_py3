#! /usr/bin/env python3

import rospy
import actionlib
from arm_motion_controller_py3.msg import handCalibrationAction, handCalibrationFeedback, handCalibrationResult
from std_msgs.msg import Float32MultiArray

import numpy as np

left_hand_pose = Float32MultiArray()
right_hand_pose = Float32MultiArray()


class ActionServer():

    def __init__(self):
        self.a_server = actionlib.SimpleActionServer(
            "hand_calibration_as", handCalibrationAction, execute_cb=self.execute_cb, auto_start=False)
        self.a_server.start()

    def execute_cb(self, goal):

        success = True
        calib_flag = True
        feedback = handCalibrationFeedback()
        result = handCalibrationResult()
        rate = rospy.Rate(1)

        if goal.calib_request:
            left_hand_pose.data = 1.0
            right_hand_pose.data = 2.0
            feedback.calib_success = calib_flag
            result.left_hand_pose.append(left_hand_pose.data)
            result.right_hand_pose.append(right_hand_pose.data)

            left_hand_pose.data = 1.0
            right_hand_pose.data = 2.0
            feedback.calib_success = calib_flag
            result.left_hand_pose.append(left_hand_pose.data)
            result.right_hand_pose.append(right_hand_pose.data)
        else:
            feedback.calib_success = False # not calib_flag
        self.a_server.publish_feedback(feedback)
        rate.sleep()

        if success:
            self.a_server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("action_server")
    s = ActionServer()
    rospy.spin()

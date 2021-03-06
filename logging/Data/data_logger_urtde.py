#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is a data_logger class. Subscribes the IMU readings and save them into a CSV fileself.

"""

# imports
from queue import Queue, Empty
import threading
import csv
import Data.data_urtde as data


# Private Globals
_DATA = Queue()
_DATA_LOGGER = None
_logging_enabled = False


# Functions
# def log_metrics(time, tee, human_wrist_angles, hand_pose, gain):
def log_metrics(time, lhand_pose, rhand_pose, hand_pose, tgoal_pose, tactual_pose, status, force_mode, force_human, lelbow, relbow, current_tcp_pose, table_acc, table_angle):
    if _DATA_LOGGER.running:
        lhand_pose_arr = [lhand_pose.position.x, lhand_pose.position.y, lhand_pose.position.z, lhand_pose.orientation.x, lhand_pose.orientation.y, lhand_pose.orientation.z, lhand_pose.orientation.w]
        rhand_pose_arr = [rhand_pose.position.x, rhand_pose.position.y, rhand_pose.position.z, rhand_pose.orientation.x, rhand_pose.orientation.y, rhand_pose.orientation.z, rhand_pose.orientation.w]
        hand_pose_arr = [hand_pose.position.x, hand_pose.position.y, hand_pose.position.z, hand_pose.orientation.x, hand_pose.orientation.y, hand_pose.orientation.z, hand_pose.orientation.w]
        tgoal_pose_arr = [tgoal_pose.position.x, tgoal_pose.position.y, tgoal_pose.position.z, tgoal_pose.orientation.x, tgoal_pose.orientation.y, tgoal_pose.orientation.z, tgoal_pose.orientation.w]
        tactual_pose_arr = [tactual_pose.position.x, tactual_pose.position.y, tactual_pose.position.z, tactual_pose.orientation.x, tactual_pose.orientation.y, tactual_pose.orientation.z, tactual_pose.orientation.w]
        status_string = status ## Don't know why not status.data?
        force_mode_string = force_mode.data
        force_human_float = force_human.data
        lelbow_data = lelbow
        relbow_data = relbow
        current_tcp_pose_arr = [current_tcp_pose.position.x, current_tcp_pose.position.y, current_tcp_pose.position.z, current_tcp_pose.orientation.x, current_tcp_pose.orientation.y, current_tcp_pose.orientation.z, current_tcp_pose.orientation.w]
        table_acc_arr = [table_acc.x, table_acc.y, table_acc.z]
        table_angle_arr = [table_angle.x, table_angle.y, table_angle.z]

        new_data = [time, lhand_pose_arr, rhand_pose_arr, hand_pose_arr, tgoal_pose_arr, tactual_pose_arr, status_string, force_mode_string, force_human_float, lelbow_data, relbow_data, current_tcp_pose_arr, table_acc_arr, table_angle_arr]
        _DATA.put(new_data)
        # print new_data
    else:
        return


def enable_logging():
    global _DATA_LOGGER
    global _logging_enabled
    _logging_enabled = True
    _DATA_LOGGER = DataLogger()  # thread here
    _DATA_LOGGER.start() ## start thread not start() module of your logger.
    print("enable_logging")


def disable_logging():
    global _logging_enabled
    if _logging_enabled:
        if _DATA_LOGGER.running:
            _DATA_LOGGER.stop()
        _logging_enabled = False
        print("disable_logging")


class DataLogger(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True
        self.filename = data.get_new_filename()
        self.fp = open(self.filename, 'w')
        self.writer = csv.writer(self.fp, lineterminator='\n')
        # write the header of the CSV file (the labels of each field/feature)
        print("labels:", data.DATA_LABELS)
        self.writer.writerow(data.DATA_LABELS)
        self.running = True

    def run(self):
        while self.running:
            try:
                row = _DATA.get(timeout=1)
                print("data:", row)
                self.writer.writerow(row)
            except Empty:
                continue
        self.close()

    def close(self):
        if self.fp is not None:
            self.fp.close()
            self.fp = None

    def stop(self):
        self.running = False

    def __del__(self):
        self.close()
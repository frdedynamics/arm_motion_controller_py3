#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64
from ros_myo.msg import EmgArray

# emg_data = []  ## change a numpy array if sum is too slow
emg_sum = 0 

def cb_emg(msg):
    global emg_sum
    emg_sum = sum(msg.data)




if __name__ == '__main__':
    rospy.init_node('emg_to_gripper', anonymous=True)
    sub_emg = rospy.Subscriber('/myo_raw/myo_emg', EmgArray, cb_emg)
    pub = rospy.Publisher('/emg_sum', Int64, queue_size=1)
    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():
        try:
            pub.publish(emg_sum)            
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
#!/usr/bin/env python3

'''
This node controls Robotiq gripper with ON/OFF commands via Myo Armband EMG data.
The EMG data is processed poorly so they are not reliable. Therefore, it works only as ON/OFF commands
The delay looks like inevitable if we want to process EMG data propely: https://www.youtube.com/watch?v=EnY56VFmAYY
'''
import rospy, sys
from std_msgs.msg import Int16, Bool
from ros_myo.msg import EmgArray

# emg_data = []  ## change a numpy array if sum is too slow
emg_sum = 0 
gripper_open = True
emg_sum_th = 3000

def cb_emg(msg):
    global emg_sum
    emg_sum = sum(msg.data)


if __name__ == '__main__':
    rospy.init_node('emg_to_gripper', anonymous=False)
    sub_emg = rospy.Subscriber('/myo_raw/myo_emg', EmgArray, cb_emg)
    pub_emg_sum = rospy.Publisher('/emg_sum', Int16, queue_size=1)
    pub_gripper = rospy.Publisher('/cmd_grip_bool', Bool, queue_size=1)
    rate = rospy.Rate(10)

    try:
        emg_sum_th = rospy.get_param("/emg_sum_th")
        print("emg_sum_th parameter set:",emg_sum_th)
    except:
        print("no emg_sum_th parameter set")

    while not rospy.is_shutdown():
        try:
            pub_emg_sum.publish(emg_sum)  
            # print(emg_sum)            
            if not emg_sum < emg_sum_th:
                if gripper_open:
                    gripper_open =  False
                    pub_gripper.publish(gripper_open)
                    rospy.sleep(2)
                else:
                    gripper_open = True
                    pub_gripper.publish(gripper_open)
                    rospy.sleep(2)

            rate.sleep()      
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

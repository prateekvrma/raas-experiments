#!/usr/bin/env python
from time import time
import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State
from mavros_msgs.msg import PositionTarget
from apriltag_ros.msg import *
import numpy as np
import time

FLIGHT_ALTITUDE = 1.0
RATE = 20  # loop rate
RADIUS = 0.5  # radius of the circle
CYCLE_S = 8  # time to complroslaunch voxl_mpa_to_ros voxl_mpa_to_ros.launchete a c1ircle in seconds
STEPS = CYCLE_S * RATE
current_state = State()
path = []  # a list of [PositionTarget()]
# generate a simple circle using parametric equation
# note this is in ENU coordinates since mavros will convert to NED
# x right, y forward, z up.



def tag_callback(msg):
	# print(msg)
    tag_detect = 0
    if not msg.detections:
        print("No Tag Detected!!!!!!")
        # return tag_detect, tag_detect
    else:
		# print(msg.detections[0].id)
		# tag_detected = True
        print("Tag Detected!")
        # rospy.sleep(5)
        detect_and_land()
        return True
    
def state_cb(msg):
    global current_state
    current_state = msg
    print('current state is {}'.format(current_state))


def detect_and_land():
    # now land
    print("-----------------Landing initiated-----------------")
    target_local_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    # the setpoint publishing rate MUST be faster than 2Hz
    rate = rospy.Rate(RATE)

    position_dir_1 = PositionTarget()
    position_dir_1.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    position_dir_1.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                              PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                              PositionTarget.IGNORE_YAW_RATE
    position_dir_1.position.x = -2
    position_dir_1.position.y = 0
    position_dir_1.position.z = 0 #FLIGHT_ALTITUDE
    position_dir_1.velocity.x = 0
    position_dir_1.velocity.y = 0
    position_dir_1.velocity.z = 0
    position_dir_1.acceleration_or_force.x = 0
    position_dir_1.acceleration_or_force.y = 0
    position_dir_1.acceleration_or_force.z = 0
    # path starts pointing 0 degrees right of forward (y axis in ENU)
    # ENU yaw is angle left (CCW) of X axis which is to the right.
    # hence yaw here is 90 degrees
    # plus 90 gets us from x axis as 0 to y axis as 0
    position_dir_1.yaw = 0
    position_dir_1.yaw_rate = 0
    for i in range(100):
        target_local_pub.publish(position_dir_1)
        rate.sleep()
    rospy.loginfo("waiting for offboard mode")
    while not rospy.is_shutdown():
        target_local_pub.publish(position_dir_1)
        rate.sleep()
        if (current_state.mode == "OFFBOARD") and current_state.armed:
            break
    # give the system 2 seconds to get to the position
    i = 5 * RATE
    rospy.loginfo("--------------Landing---------------")
    while (not rospy.is_shutdown()) and (i > 0):
        # Update timestamp and publish position target
        target_local_pub.publish(position_dir_1)
        i += 1
        if i >= STEPS:
            i = 0
        rate.sleep()
    print("Task Completed...")

def main():
    rospy.init_node('apr_test_node', anonymous=True)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_callback)
    # print("End")

    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

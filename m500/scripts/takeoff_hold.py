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

FLIGHT_ALTITUDE = 2.0
RATE = 20  # loop rate
RADIUS = 0.5  # radius of the circle
CYCLE_S = 8  # time to complete the path in seconds
STEPS = CYCLE_S * RATE
current_state = State()
path = []  # a list of [PositionTarget()]


def state_cb(msg):
    global current_state
    current_state = msg
    print('current state is {}'.format(current_state))

def main():
    rospy.init_node('takeoff_hold_node', anonymous=True)
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    target_local_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    # the setpoint publishing rate MUST be faster than 2Hz
    rate = rospy.Rate(RATE)

    position_home = PositionTarget()
    position_home.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    position_home.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                              PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                              PositionTarget.IGNORE_YAW_RATE
    position_home.position.x = 0
    position_home.position.y = 0
    position_home.position.z = FLIGHT_ALTITUDE
    position_home.velocity.x = 0
    position_home.velocity.y = 0
    position_home.velocity.z = 0
    position_home.acceleration_or_force.x = 0
    position_home.acceleration_or_force.y = 0
    position_home.acceleration_or_force.z = 0
    position_home.yaw = 0
    position_home.yaw_rate = 0
    
    ######
    # send a few setpoints before starting
    for i in range(100):
        target_local_pub.publish(position_home)
        rate.sleep()
    rospy.loginfo("waiting for offboard mode")
    while not rospy.is_shutdown():
        target_local_pub.publish(position_home)
        rate.sleep()
        if (current_state.mode == "OFFBOARD") and current_state.armed:
            break
    # give the system 2 seconds to get to home position
    i = 2 * RATE
    rospy.loginfo("going home")
    while (not rospy.is_shutdown()) and (i > 0):
        i -= 1
        target_local_pub.publish(position_home)
        rate.sleep()

    print("End")

    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

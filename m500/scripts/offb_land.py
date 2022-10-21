#!/usr/bin/env python
import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State
from mavros_msgs.msg import PositionTarget
import numpy as np

FLIGHT_ALTITUDE = 1.0
RATE = 20  # loop rate
RADIUS = 0.5  # radius of the circle
CYCLE_S = 8  # time to complete the path in seconds
STEPS = CYCLE_S * RATE
current_state = State()
path = []  # a list of [PositionTarget()]

def main():
    rospy.init_node('Landing', anonymous=True)
    target_local_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    # the setpoint publishing rate MUST be faster than 2Hz
    rate = rospy.Rate(RATE)

    position_dir = PositionTarget()
    position_dir.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    position_dir.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                              PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                              PositionTarget.IGNORE_YAW_RATE
    position_dir.position.x = -2
    position_dir.position.y = 0
    position_dir.position.z = 1
    position_dir.velocity.x = 0
    position_dir.velocity.y = 0
    position_dir.velocity.z = 0
    position_dir.acceleration_or_force.x = 0
    position_dir.acceleration_or_force.y = 0
    position_dir.acceleration_or_force.z = 0
    # path starts pointing 0 degrees right of forward (y axis in ENU)
    # ENU yaw is angle left (CCW) of X axis which is to the right.
    # hence yaw here is 90 degrees
    # plus 90 gets us from x axis as 0 to y axis as 0
    position_dir.yaw = 0
    position_dir.yaw_rate = 0
    
    ######

    position_dir_1 = PositionTarget()
    position_dir_1.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    position_dir_1.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                               PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                               PositionTarget.IGNORE_YAW_RATE
    position_dir_1.position.x = 0
    position_dir_1.position.y = 0
    position_dir_1.position.z = 0
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
    
    ######
    i = 0
    rospy.loginfo("back path")
    while not rospy.is_shutdown():
        target_local_pub.publish(position_dir)
        i += 1
        if i >= STEPS:
            i = 0
        rate.sleep()
    rospy.loginfo("land path")
    while not rospy.is_shutdown():
        target_local_pub.publish(position_dir_1)
        i += 1
        if i >= STEPS:
            i = 0
        rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

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

def state_cb(msg):
    global current_state
    current_state = msg
    print('current state is {}'.format(current_state))


def main():
    rospy.init_node('vio_test_node', anonymous=True)
    robot_local_pose = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    rate = rospy.Rate(RATE)

    # wait for FCU connection
    # while ((not rospy.is_shutdown()) and (not current_state.connected)):
    #     rate.sleep()
    #     rospy.loginfo("connecting to FCU...")
    # keep this pose constant, home position
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
    # path starts pointing 0 degrees right of forward (y axis in ENU)
    # ENU yaw is angle left (CCW) of X axis which is to the right.
    # hence yaw here is 90 degrees
    # plus 90 gets us from x axis as 0 to y axis as 0
    position_home.yaw = 0
    position_home.yaw_rate = 0
    
    ######
    position_dir = PositionTarget()
    position_dir.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    position_dir.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                              PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                              PositionTarget.IGNORE_YAW_RATE
    position_dir.position.x = 0
    position_dir.position.y = 0
    position_dir.position.z = 0 #FLIGHT_ALTITUDE
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
    
    #####################################################################################################################3

    
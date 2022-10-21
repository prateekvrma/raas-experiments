#!/usr/bin/env python

# ROS python and MAVROS API
import rospy
import mavros

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

import numpy as np

FLIGHT_ALTITUDE = 1.0
RATE = 20  # loop rate

STEPS = 0

current_state = State()
path = []  # a list of [PositionTarget()]

# generate a simple circle using parametric equation
# note this is in ENU coordinates since mavros will convert to NED
# x right, y forward, z up.

def init_path():
    global path
    global STEPS
    Points = [(5.0, 5.0),(-5.0, 5.0),(-5.0, -5.0), (5.0, -5.0)]#[(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)]
    trajectory = []
    uav_speed = 1
    rate = 20
    for i in range(len(Points)-1):
        dis = np.linalg.norm(np.array(Points[i]) - np.array(Points[i+1]))
        if dis < 1e-6:
            continue
        time = (dis/uav_speed)
        steps = round(rate*time)
        steps = int(steps)
        STEPS += steps
        delta_x = (Points[i+1][0] - Points[i][0])/steps
        delta_y = (Points[i+1][1] - Points[i][1])/steps
        
        if delta_x < 1e-3:
            X = np.ones((steps,))*Points[i][0]
        else:
            X = np.arange(float(Points[i][0]), float(Points[i+1][0]), delta_x)
        
        if delta_y < 1e-3:
            Y = np.ones((steps,))*Points[i][1]
        else:
            Y = np.arange(float(Points[i][1]), float(Points[i+1][1]), delta_y)
        
        assert len(X) == len(Y) == steps
        for j in range(len(X)):
            way_point = PositionTarget()
            # print(way_point)
            way_point.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            way_point.type_mask = 0

            way_point.position.x = X[j]
            way_point.position.y = Y[j]
            way_point.position.z = FLIGHT_ALTITUDE
            path.append(way_point)



    # for i in range(STEPS):
    #     way_point = PositionTarget()
    #     # print(way_point)
    #     way_point.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    #     way_point.type_mask = 0

    #     way_point.position.x = 1
    #     way_point.position.y = 0
    #     way_point.position.z = FLIGHT_ALTITUDE
    #     path.append(way_point)

def state_cb(msg):
    global current_state
    current_state = msg
    print('current state is {}'.format(current_state))        

def main():
    rospy.init_node('straight_line', anonymous=True)
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    land_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    target_local_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)

    # the setpoint publishing rate MUST be faster than 2Hz
    rate = rospy.Rate(RATE)

    position_home = PositionTarget()
    position_home.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    position_home.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                              PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                              PositionTarget.IGNORE_YAW_RATE
    position_home.position.x = 5
    position_home.position.y = 5
    position_home.position.z = FLIGHT_ALTITUDE
    
    # path starts pointing 0 degrees right of forward (y axis in ENU)
    # ENU yaw is angle left (CCW) of X axis which is to the right.
    # hence yaw here is 90 degrees
    # plus 90 gets us from x axis as 0 to y axis as 0
    position_home.yaw = 0
    position_home.yaw_rate = 0
    init_path()
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
    
     # now begin motion
    i = 0
    rospy.loginfo("following path")
    while not rospy.is_shutdown():
        # Update timestamp and publish position target
        path[i].header.stamp = rospy.Time.now()
        target_local_pub.publish(path[i])
        print(path[i])
        i += 1
        if i >= STEPS:
            i = 0
        rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




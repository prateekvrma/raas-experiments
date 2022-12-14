/**
 * @file mavros_test_node.cpp
 *
 * Copyright (c) 2020 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * For a license to use on non-ModalAI hardware, please contact
 * license@modalai.com
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <math.h>

#define RATE            20  // loop rate hz
#define RADIUS          6.0 // radius of figure 8 in meters
#define D_ALTITUDE      8.0f
#define CYCLE_S         4   // time to complete one figure 8 cycle in seconds
#define CYCLE_Z         20  // time to go up and back down
#define XY_STEPS        (CYCLE_S*RATE)
#define Z_STEPS         (CYCLE_Z*RATE)
#define LAPS            4

#define FLIGHT_ALTITUDE 1.0f
#define PI  3.14159265358979323846264338327950

#define ALL_STEPS       (2*LAPS*XY_STEPS)
float ALT_STEP = (D_ALTITUDE - FLIGHT_ALTITUDE)/(ALL_STEPS/2);

mavros_msgs::State current_state;
mavros_msgs::PositionTarget path[ALL_STEPS];

// generate a path following Bernoulli's lemiscate as a parametric equation
// note this is in ENU coordinates since mavros will convert to NED
// x right, y forward, z up.
void init_path()
{
    int i;
    const double dt = 1.0/RATE;
    const double dadt = (2.0*PI)/CYCLE_S; // first derivative of angle with respect to time
    const double r = RADIUS;
    double r2 = RADIUS;

    for(i=0;i<ALL_STEPS;i++){
        // basic fields in the message
        path[i].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        path[i].type_mask = 0;

        // calculate the parameter a which is an angle sweeping from -pi/2 to 3pi/2
        // through the curve
        double a = (i%XY_STEPS)*(2.0*PI/XY_STEPS);
        double az = i*(2.0*PI/Z_STEPS);
        double c = cos(a);
        double cz = cos(az);
        double c2a = cos(2.0*a);
        double c4a = cos(4.0*a);
        double c2am3 = c2a-3.0;
        double s = sin(a);
        double sz = sin(az);
        double cc = c*c;
        double ss = s*s;
        double sspo = (s*s)+1.0; // sin squared plus one
        double ssmo = (s*s)-1.0; // sin squared minus one
        double sspos = sspo*sspo;

        if(i < 0.5*ALL_STEPS)
        {
	    r2 = r*(i/(ALL_STEPS*0.5));
            // Position
            // https://www.wolframalpha.com/input/?i=%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
            // path[i].position.x =  (r*c)   / sspo;
            path[i].position.x = r2*c;
            // https://www.wolframalpha.com/input/?i=%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
            // path[i].position.y = -(r*c*s) / sspo;
            path[i].position.y = r2*s;
            // path[i].position.z =  FLIGHT_ALTITUDE;
            // path[i].position.z =  FLIGHT_ALTITUDE + (1-cz)/2*D_ALTITUDE;
            path[i].position.z =  FLIGHT_ALTITUDE + i*ALT_STEP;

            // Velocity
            // https://www.wolframalpha.com/input/?i=derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
            path[i].velocity.x =  -r2*s;
            // https://www.wolframalpha.com/input/?i=derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
            path[i].velocity.y =   r2*c;
            path[i].velocity.z =  ALT_STEP;

            // Acceleration
            // https://www.wolframalpha.com/input/?i=second+derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
            path[i].acceleration_or_force.x =  -r2*c;
            // see https://www.wolframalpha.com/input/?i=second+derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
            path[i].acceleration_or_force.y =  -r2*s;
            path[i].acceleration_or_force.z =  0;

            // calculate yaw as direction of velocity
            // plus pi/2 since ROS yaw=0 lines up with x axis with points out to
            // the right, not forward along y axis
            path[i].yaw = PI + a;
        }
        else
        {
            path[i].position.x = 0;
            path[i].position.y = 0;
            path[i].position.z =  D_ALTITUDE - (i - 0.5*ALL_STEPS)*ALT_STEP;
            
            path[i].velocity.x = 0;
            path[i].velocity.y = 0;
            path[i].velocity.z = -ALT_STEP;
            
            path[i].acceleration_or_force.x =  0;
            path[i].acceleration_or_force.y =  0;
            path[i].acceleration_or_force.z =  0;

            path[i].yaw = PI;
        }

        printf("x:%7.3f y:%7.3f z:%7.3f yaw:%7.1f\n", path[i].position.x, path[i].position.y, path[i].position.z, path[i].yaw*180.0f/PI);
    }

    // calculate yaw_rate by dirty differentiating yaw
    for(i=0;i<ALL_STEPS;i++){
        double next = path[(i+1)%ALL_STEPS].yaw;
        double curr = path[i].yaw;
        // account for wrap around +- PI
        if((next-curr) < -PI) next+=(2.0*PI);
        if((next-curr) >  PI) next-=(2.0*PI);
        path[i].yaw_rate = (next-curr)/dt;
    }

}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    int i;

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub           = nh.subscribe<mavros_msgs::State>
                                        ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub        = nh.advertise<geometry_msgs::PoseStamped>
                                        ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client    = nh.serviceClient<mavros_msgs::CommandBool>
                                        ("mavros/cmd/arming");
    ros::ServiceClient land_client      = nh.serviceClient<mavros_msgs::CommandTOL>
                                        ("mavros/cmd/land");
    ros::ServiceClient set_mode_client  = nh.serviceClient<mavros_msgs::SetMode>
                                        ("mavros/set_mode");
    ros::Publisher target_local_pub     = nh.advertise<mavros_msgs::PositionTarget>
                                        ("mavros/setpoint_raw/local", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(RATE);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("\rconnecting to FCT...");
    }

    // keep this pose constant, home position
    mavros_msgs::PositionTarget position_home;
    position_home.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    position_home.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    position_home.position.x = 0;
    position_home.position.y = 0;
    position_home.position.z = FLIGHT_ALTITUDE;
    position_home.velocity.x = 0;
    position_home.velocity.y = 0;
    position_home.velocity.z = 0;
    position_home.acceleration_or_force.x = 0;
    position_home.acceleration_or_force.y = 0;
    position_home.acceleration_or_force.z = 0;
    // path starts pointing 45 degrees right of forward (y axis in ENU)
    // ENU yaw is angle left (CCW) of X axis which is to the right.
    // hence yaw here is -45 degrees
    // plus 90 gets us from x axis as 0 to y axis as 0
    position_home.yaw = (-45.0f + 90.0f) * PI / 180.0f;
    position_home.yaw_rate = 0;

    init_path();

    //send a few setpoints before starting
    for(i = 100; ros::ok() && i > 0; --i){
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
    }

HOME:
    ROS_INFO("waiting for offboard mode");
    // wait for the system to be armed and in offboard mode
    while(ros::ok()){
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
        if(current_state.mode == "OFFBOARD" && current_state.armed) break;
    }

    // give the system 2 seconds to get to home position
    i = RATE * 2;
    ROS_INFO("going home");
    while(ros::ok() && i>0){
        // return to home position if px4 falls out of offboard mode or disarms
        if(current_state.mode != "OFFBOARD" || !current_state.armed){
            goto HOME;
        }
        i--;
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
    }

    // now begin figure 8 path,
    i=0;
    ROS_INFO("following path");
    while(ros::ok()){
        // return to home position if px4 falls out of offboard mode or disarms
        if(current_state.mode != "OFFBOARD" || !current_state.armed){
            goto HOME;
        }
        target_local_pub.publish(path[i]);
    i++;
        if(i>=ALL_STEPS) i=0;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

// /* HOW TO ARM , LAND AND CHANGE MODE

//     mavros_msgs::SetMode offb_set_mode;
//     offb_set_mode.request.custom_mode = "OFFBOARD";

//     mavros_msgs::CommandBool arm_cmd;
//     arm_cmd.request.value = true;

//     mavros_msgs::CommandTOL land_cmd;
//     land_cmd.request.yaw = 0;
//     land_cmd.request.latitude = 0;
//     land_cmd.request.longitude = 0;
//     land_cmd.request.altitude = 0;

//     if( set_mode_client.call(offb_set_mode) &&
//         offb_set_mode.response.mode_sent){
//         ROS_INFO("Offboard enabled");
//     }

//     if( arming_client.call(arm_cmd) &&
//         arm_cmd.response.success){
//         ROS_INFO("Vehicle armed");
//     }

//         ROS_INFO("tring to land");
//     while (!(land_client.call(land_cmd) &&
//             land_cmd.response.success)){
//       //local_pos_pub.publish(pose);
//       ROS_INFO("tring to land");

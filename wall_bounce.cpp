/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <gazebo_msgs/ContactsState.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double force1 = 0;
double force2 = 0;

void force_cb1(const gazebo_msgs::ContactsState::ConstPtr& msg){
    if(!msg->states.empty()){
    force1 = msg->states[0].total_wrench.force.z;
    }
}

void force_cb2(const gazebo_msgs::ContactsState::ConstPtr& msg){
    if(!msg->states.empty()){
    force2 = msg->states[0].total_wrench.force.z;
    }
}

double pos_x, pos_y, pos_z;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pos_x = msg->pose.position.x;
    pos_y = msg->pose.position.y;
    pos_z = msg->pose.position.z;
}

bool takeoff = 1;
bool forward = 0;
bool backward = 0;
bool land = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber force_sub1 = nh.subscribe<gazebo_msgs::ContactsState>
            ("contact_sensor_state_1", 10, force_cb1);
    ros::Subscriber force_sub2 = nh.subscribe<gazebo_msgs::ContactsState>
            ("contact_sensor_state_2", 10, force_cb2);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offb_set_mode.request.base_mode = 0;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(current_state.mode == "OFFBOARD" && current_state.armed){
                if(takeoff && (pos_z < 0.95)){
                local_pos_pub.publish(pose);
                ROS_INFO("Vehicle takeoff");
                }
                else{
                    takeoff = 0;
                    forward = 1;
                }

                mavros_msgs::PositionTarget target;

                if(!takeoff && forward && (force1>=-0.1) && (force2>=-0.1)){
                    target.header.frame_id = "map";
                    target.coordinate_frame = 1;
                    target.type_mask = 0xBC7; //vx, vy, z, yaw
                    target.velocity.z = 1*(1-pos_z);
                    target.velocity.y = 0;
                    target.velocity.x = 0.5;
                    target.yaw = 0;
                    setpoint_raw_pub.publish(target);
                    ROS_INFO("Vehicle forward");
                }
                else{
                    forward = 0;
                    backward = 1;
                }

                if(!takeoff && !forward && backward && pos_x > -0.1){
                    target.header.frame_id = "map";
                    target.coordinate_frame = 1;
                    target.type_mask = 0xBC7; //vx, vy, z, yaw
                    target.velocity.z = 1*(1-pos_z);
                    target.velocity.y = 0;
                    target.velocity.x = -0.5;
                    target.yaw = 0;
                    setpoint_raw_pub.publish(target);
                    ROS_INFO("Vehicle touched wall, bounce back");
                }
                else{
                    backward = 0;
                    land =1;
                }

                if(!takeoff && !forward && !backward && land){
                    pose.pose.position.x = -0.2;
                    pose.pose.position.y = 0;
                    pose.pose.position.z = -0.2;
                    if (pos_z > 0.0){
                        local_pos_pub.publish(pose);
                        ROS_INFO("Vehicle land");
                    }
                    else{
                        arm_cmd.request.value = false;
                        if( arming_client.call(arm_cmd) &&
                            arm_cmd.response.success){
                            ROS_INFO("Vehicle disarmed");
                            break;
                        }
                    }
                }
        }
        else{
            local_pos_pub.publish(pose);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
    ros::shutdown();
}
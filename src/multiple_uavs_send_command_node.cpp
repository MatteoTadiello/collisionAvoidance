/**
 * @file multiple_uavs_send_command_node.cpp
 * @brief Offboard control example node, for multiple UAVs takeoff 
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <vector>

#include "Vector3.h"
#include "Agent.h"
#include "Definitions.h"
#include "KdTree.h"

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "multiple_uavs_send_command_node" );
	ros::NodeHandle nh;

    if( argc != 2 ){
         ROS_INFO("Il comando deve essere chiamato con un parametro che indica il numero di UAVs");
         return 1;
    }

    std::vector<ORCA::Agent *> drones(atoi(argv[1]));
    std::vector<ros::Subscriber> subscribers(atoi(argv[1]));
    std::vector<ros::Subscriber> positions(atoi(argv[1]));
    std::vector<ros::Subscriber> velocities(atoi(argv[1]));
    std::vector<ros::ServiceClient> arming_clients(atoi(argv[1]));
    std::vector<ros::ServiceClient> set_mode_clients(atoi(argv[1]));
    std::vector<ros::Publisher> local_pos_pub(atoi(argv[1]));

	std::string mavros = "mavros"; 
	for(int i = 1 ; i <= atoi(argv[1]) ; i++ ){
		subscribers[i-1] = nh.subscribe<mavros_msgs::State>
            (mavros+(std::to_string(i))+"/state", 10, state_cb);

        positions[i-1] = nh.subscribe<sensor_msgs::NavSatFix>
            (mavros+(std::to_string(i))+"/global_position/global",10, &ORCA::Agent::position_cb, drones[i-1]);
    	velocities[i-1] = nh.subscribe<geometry_msgs::TwistStamped>
            (mavros+(std::to_string(i))+"global_position/gp_vel",10, &ORCA::Agent::vel_cb, drones[i-1]);
        local_pos_pub[i-1] = nh.advertise<geometry_msgs::PoseStamped>
            (mavros+(std::to_string(i))+"/setpoint_position/local", 10);
        //ROS_INFO_STREAM(positions[i-1]);
    	arming_clients[i-1] = nh.serviceClient<mavros_msgs::CommandBool>
            (mavros+(std::to_string(i))+"/cmd/arming");
    	set_mode_clients[i-1] = nh.serviceClient<mavros_msgs::SetMode>
            (mavros+(std::to_string(i))+"/set_mode");
	}

    //ROS_INFO_STREAM(positions[1]);

	//the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    int n;

    nh.getParam("mavros1/system_id",n);
    sensor_msgs::NavSatFix pos;

    std::vector<geometry_msgs::PoseStamped> pose(atoi(argv[1]));

    for(int i=0 ; i < atoi(argv[1]) ; i++ ){
        pose[i].pose.position.z = 5;
        if(i%2==0){
            pose[i].pose.position.x = 0+i;
            pose[i].pose.position.y = 0;
        }else{
            pose[i].pose.position.x = 0+i-1;
            pose[i].pose.position.y = 2;
        }

    }



    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        for(int i = 0 ; i < atoi(argv[1]) ; i++ ){
            local_pos_pub[i].publish(pose[i]);
        }
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int timer = 0;
    bool change = false;
    while(ros::ok()){
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                for(int i = 0 ; i < atoi(argv[1]) ; i++ ){
                    if( set_mode_clients[i].call(offb_set_mode) &&
                        offb_set_mode.response.mode_sent){
                        ROS_INFO("Offboard enabled");
                    }
                }
                
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                        for(int i = 0 ; i < atoi(argv[1]) ; i++ ){
                            if( arming_clients[i].call(arm_cmd) &&
                            arm_cmd.response.success){
                            ROS_INFO("Vehicle armed");
                        }
                    }
                    
                    last_request = ros::Time::now();
                }
            }


            if(ros::Time::now() - last_request > ros::Duration(5.0)){
                if(change){
                    change=false;
                }else{
                    change=true;
                }
            }

            ROS_INFO_STREAM("ilvalore di change e: " << change);
            if(change){
                for(int i = 100; ros::ok() && i > 0; --i){
                for(int i = 0 ; i < atoi(argv[1]) ; i++ ){
                    local_pos_pub[i].publish(pose[i]);
                }}
            }else{
                for(int i = 100; ros::ok() && i > 0; --i){
                for(int i = atoi(argv[1])-1 ; i > 0 ; i-- ){
                    local_pos_pub[atoi(argv[1])-1-i].publish(pose[i]);
                }}
            }
            ros::spinOnce();
        rate.sleep();
            
            ++timer;
        } 

    return 0;
}
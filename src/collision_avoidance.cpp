/**
 * @file collision_avoidace.cpp
 * @brief Offboard control for multiple UAVs collision avoidance 
 */

#include <ros/ros.h>
#include <string>
#include <vector>



#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include "Vector3.h"
#include "Definitions.h"
#include "KdTree.h"
#include "Agent.h"

const int maxNeighbors = 10;
const float maxSpeed = 1.5f;
const float neighborDist = 1.0f;
const float radius = 0.3;
const float timeHorizon = 10.0f;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "collision_avoidance_node" );
	ros::NodeHandle nh;

	ORCA::KdTree tree;

    if( argc != 2 ){
         ROS_INFO("Il comando deve essere chiamato con un parametro che indica il numero di UAVs");
         return 1;
    }

    int num = atoi(argv[1]); // Numero di droni nella simulazione
    //DEFINISCO IL VETTORE di droni e tutti i publisher subscriber utili all'esecuzione
    std::vector<ORCA::Agent> drones(num);
    std::vector<ORCA::Agent *> drones_pntr(num);
    std::vector<ros::Subscriber> states(num); 		//Vettori degli stati dei droni
    std::vector<ros::Subscriber> positions(num); 		//Vettore per avere le posizioni
   	std::vector<ros::Subscriber> velocities_sub(num); //Vettore per avere le velocita'
	std::vector<ros::Publisher> altitude(num);         //Vettore per pubblicare l'altezza alla quale decollare
	std::vector<ros::Publisher> velocities_pub(num); 	//Vettore per pubblicare la velocita'ad ogni drone   
    std::vector<ros::ServiceClient> set_mode_clients(num);
    std::vector<ros::ServiceClient> arming_clients(num);

    for(int i=0 ; i < num; i++){
    	drones_pntr[i]=&drones[i];
    }
    std::string mavros = "mavros";
    //Sottoscrivo i vettori creati sopra ai sensori su cui ricevere e pubblicare i messaggi 
    for(int i=0 ; i < num; i++){
    	states[i] = nh.subscribe<mavros_msgs::State>
    		(mavros+(std::to_string(i+1))+"/state",1,state_cb);
    	positions[i] = nh.subscribe<geometry_msgs::PoseStamped>
    		(mavros+(std::to_string(i+1))+"/local_position/pose",1, &ORCA::Agent::position_cb , &drones[i]); //
    	velocities_sub[i] = nh.subscribe<geometry_msgs::TwistStamped>
    		(mavros+(std::to_string(i+1))+"/local_position/velocity",1, &ORCA::Agent::vel_cb, &drones[i]);
    	altitude[i] =nh.advertise<geometry_msgs::PoseStamped>
    		(mavros+(std::to_string(i+1))+"/setpoint_position/local",10);
    	velocities_pub[i] = nh.advertise<geometry_msgs::Twist>
    		(mavros+(std::to_string(i+1))+"/setpoint_velocity/cmd_vel_unstamped",1);
    	arming_clients[i] = nh.serviceClient<mavros_msgs::CommandBool>
            (mavros+(std::to_string(i+1))+"/cmd/arming");
    	set_mode_clients[i] = nh.serviceClient<mavros_msgs::SetMode>
            (mavros+(std::to_string(i+1))+"/set_mode");
    	drones[i].id_=i;
        drones[i].system_id=i;
    	drones[i].maxNeighbors_=maxNeighbors;
    	drones[i].maxSpeed_=maxSpeed;
    	drones[i].neighborDist_=neighborDist;
    	drones[i].radius_=radius;
    	drones[i].timeHorizon_=timeHorizon;
        drones[i].tree_= &tree;
    }
    tree.setAgents(drones_pntr);
    ros::Rate rate(20.0); //0.05 sec
    tree.buildAgentTree();


    while(ros::ok &&!current_state.connected){
    	ros::spinOnce();
    	rate.sleep();
    }

    std::vector<geometry_msgs::PoseStamped> goals(num);


        
    for(int i=0; i<num ; i++){
        goals[i].pose.position.x = (2*(num-1))-((i*2)+((i)*2));
        goals[i].pose.position.y = 0;
        goals[i].pose.position.z = 10;
        std::cout << "mavros" << i+1 <<"\n\nGoals : " <<  goals[i].pose.position << std::endl;

    }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
    	for(int j=0 ; j<num;j++){
    		altitude[j].publish(goals[j]);
    	}
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
    	//Per abilitare l'offboard e fare decollare i droni
    	if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
    		for(int i=0; i<num ; i++){
    			if( set_mode_clients[i].call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                	ROS_INFO("Offboard enabled");
            }}
            
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
            	for(int i=0; i<num ; i++){
                	if( arming_clients[i].call(arm_cmd) &&
                    arm_cmd.response.success){
                    	ROS_INFO("Vehicle armed");
                }}
                last_request = ros::Time::now();
            }
        }

        for(int i=0; i<num;i++){
        	altitude[i].publish(goals[i]);
        }

        tree.buildAgentTree();


        //Calcolo i vicini e la nuova posizione
        for(int i=0; i<num; i++){

         	drones[i].computeNeighbors();
       	 	drones[i].computeNewVelocity();
            drones[i].update();

            ros::spinOnce();
            rate.sleep();
        }

        //Invio la nuova velocita' calcolata
         for(int i=0; i<num; i++){
         	velocities_pub[i].publish(drones[i].newVelocityToPublish);
            ROS_INFO_STREAM( "mavros" << i << " " << drones[i].newVelocityToPublish << std::endl);
            ros::spinOnce();
            rate.sleep();
        }

        

        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}
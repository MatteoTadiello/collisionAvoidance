/**
 * @file cerchio.cpp
 * @brief Offboard control for multiple UAVs collision avoidance in a circle disposition
 */

#include <ros/ros.h>
#include <string>
#include <vector>


#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include "Vector3.h"
#include "Definitions.h"
#include "KdTree.h"
#include "Agent.h"

const int maxNeighbors = 12;
const float maxSpeed = 0.8f;
const float neighborDist = 1.5f;
const float radius = 1.5f;
const float timeHorizon = 10.0f;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
}

bool swap(bool x){
	if(x){
		return false;
	}else{
		return true;
	}
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "collision_avoidance_node" );
	ros::NodeHandle nh;

	ORCA::KdTree tree;

    int num = 12; // Numero di droni nella simulazione

    std::vector<geometry_msgs::PoseStamped> first_goal(num); // primo punto da raggiungere
    std::vector<geometry_msgs::PoseStamped> second_goal(num); // secondo punto da raggiungere

    // Imposto a 10 l'altezza di volo di tutti i droni in tutti i punti
    for(int i=0 ; i<num ; i++){
    	first_goal[i].pose.position.z = 10;
    	second_goal[i].pose.position.z = 10;
    }

    //Definisco i punti per posizionare i droni lungo la circonferenza (posizione iniziale)
    first_goal[0].pose.position.x = 0;  
    first_goal[0].pose.position.y = 10;

    first_goal[1].pose.position.x = -1;
    first_goal[1].pose.position.y = 14;

    first_goal[2].pose.position.x =  -3;
    first_goal[2].pose.position.y = 6;

    first_goal[3].pose.position.x =  -1;
    first_goal[3].pose.position.y = 18;

    first_goal[4].pose.position.x = -2; 
    first_goal[4].pose.position.y = 1;

    first_goal[5].pose.position.x =  0;
    first_goal[5].pose.position.y =0;

    first_goal[6].pose.position.x =  -2;
    first_goal[6].pose.position.y =20;

    first_goal[7].pose.position.x =  0;
    first_goal[7].pose.position.y =19;

    first_goal[8].pose.position.x =  1;
    first_goal[8].pose.position.y = 2;

    first_goal[9].pose.position.x =  1;
    first_goal[9].pose.position.y = 14;

    first_goal[10].pose.position.x = -1;
    first_goal[10].pose.position.y = 6;

    first_goal[11].pose.position.x = -2;
    first_goal[11].pose.position.y = 10;

	//Definisco i punti per posizionare i droni lungo la circonferenza (posizione finale)
    second_goal[0].pose.position.x = 20;  
    second_goal[0].pose.position.y = 10;

    second_goal[1].pose.position.x = 17;
    second_goal[1].pose.position.y = 6;

    second_goal[2].pose.position.x = 15;
    second_goal[2].pose.position.y = 14;

    second_goal[3].pose.position.x = 11;
    second_goal[3].pose.position.y = 2;

    second_goal[4].pose.position.x = 6; 
    second_goal[4].pose.position.y = 19;

    second_goal[5].pose.position.x = 0;
    second_goal[5].pose.position.y = 20;

    second_goal[6].pose.position.x = -2;
    second_goal[6].pose.position.y = 0;

    second_goal[7].pose.position.x = -8;
    second_goal[7].pose.position.y = 1;

    second_goal[8].pose.position.x = -11;
    second_goal[8].pose.position.y = 18;

    second_goal[9].pose.position.x = 17;
    second_goal[9].pose.position.y = 6;

    second_goal[10].pose.position.x = -19;
    second_goal[10].pose.position.y = 14;

    second_goal[11].pose.position.x = -22;
    second_goal[11].pose.position.y = 10;



    std::vector<ORCA::Agent> drones(num);
    std::vector<ORCA::Agent *> drones_pntr(num);
    std::vector<ros::Subscriber> states(num); 		//Vettori degli stati dei droni
    std::vector<ros::Subscriber> positions_sub(num); 		//Vettore per avere le posizioni
    std::vector<ros::Publisher> positions_pub(num);         //Vettore per pubblicare le posizioni
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
    	positions_sub[i] = nh.subscribe<geometry_msgs::PoseStamped>
    		(mavros+(std::to_string(i+1))+"/local_position/pose",1, &ORCA::Agent::position_cb , &drones[i]); //
    	positions_pub[i] =nh.advertise<geometry_msgs::PoseStamped>
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

    for(int i = 100; ros::ok() && i > 0; --i){
    	for(int j=0 ; j<num;j++){
    		positions_pub[j].publish(first_goal[j]);
    	}
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    //bool change = false;
    int count = 0 ; 


    while(ros::ok() && count < 7){
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

        for(int i = 100; ros::ok() && i > 0; --i){
    		for(int j=0 ; j<num;j++){
    			positions_pub[j].publish(first_goal[j]);
    		}
        	ros::spinOnce();
        	rate.sleep();
    	}
        count++;
        ros::spinOnce();
        rate.sleep();
        // int c=0;
        // for(int i=3; i<num ; i++){
        //     if( (drones[i].position_[0] <= (first_goal[i].pose.position.x+0.5) && drones[i].position_[0] >= (first_goal[i].pose.position.x-0.5)) 
        //     	|| (drones[i].position_[1] <= (first_goal[i].pose.position.y+0.5) && drones[i].position_[1] >= (first_goal[i].pose.position.y-0.5)) 
        //     	|| (drones[i].position_[3] <= (first_goal[i].pose.position.z+1) && drones[i].position_[3] >= (first_goal[i].pose.position.z-1))){
        //     	std::cout << "Sono "<< i+1 << std::endl;
        //     	std::cout << "sono nello stesso posto? "<< ((drones[i].position_[0] < first_goal[i].pose.position.x+0.5) && drones[i].position_[0] >= (first_goal[i].pose.position.x-0.5)) << std::endl;
        //     	std::cout << "Sono : " << drones[i].position_[0]<<std::endl;
        //     	std::cout << "Devo andare: "<< first_goal[i].pose.position.x <<std:: endl;
        //     	std::cout << c++ << std::endl;
        //     }
        // }
        // if(c==0){
        // 	change=swap(change);
        // 	std::cout << "CAMBIOOOOO" << std::endl;

        // }
         std::cout << count << std::endl;
    }

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

        //if(change){
        for(int i = 100; ros::ok() && i > 0; --i){
    		for(int j=0 ; j<num;j++){
    			positions_pub[j].publish(second_goal[j]);
    		}
    		ros::spinOnce;
    		rate.sleep();
       	}
	
       // }else{
        //	for(int i = 100; ros::ok() && i > 0; --i){
    	//		for(int j=0 ; j<num;j++){
    	//			positions_pub[j].publish(first_goal[j]);
    	//		}
        //		ros::spinOnce();
        //		rate.sleep();
    	//	}
       // }


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
            //std::cout << "mavros" << i << " " << drones[i].newVelocityToPublish << std::endl;
            ros::spinOnce();
            rate.sleep();
        }
        //count++;
        ros::spinOnce();
        rate.sleep();
        // int c=0;
        // for(int i=3; i<num ; i++){
        //     if( (drones[i].position_[0] <= (first_goal[i].pose.position.x+0.5) && drones[i].position_[0] >= (first_goal[i].pose.position.x-0.5)) 
        //     	|| (drones[i].position_[1] <= (first_goal[i].pose.position.y+0.5) && drones[i].position_[1] >= (first_goal[i].pose.position.y-0.5)) 
        //     	|| (drones[i].position_[3] <= (first_goal[i].pose.position.z+1) && drones[i].position_[3] >= (first_goal[i].pose.position.z-1))){
        //     	std::cout << "Sono "<< i+1 << std::endl;
        //     	std::cout << "sono nello stesso posto? "<< ((drones[i].position_[0] < first_goal[i].pose.position.x+0.5) && drones[i].position_[0] >= (first_goal[i].pose.position.x-0.5)) << std::endl;
        //     	std::cout << "Sono : " << drones[i].position_[0]<<std::endl;
        //     	std::cout << "Devo andare: "<< first_goal[i].pose.position.x <<std:: endl;
        //     	std::cout << c++ << std::endl;
        //     }
        // }
        // if(c==0){
        // 	change=swap(change);
        // 	std::cout << "CAMBIOOOOO" << std::endl;

        // }
         //std::cout << count << std::endl;
    }

	return 0;
}
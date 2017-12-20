#include <ros/ros.h>
#include <geometry_msgs/Twist.h> //guardar los valores de x, y, y theta
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include "gazebo_msgs/LinkStates.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
//using namespace std; 


std_msgs::String current_state; 
double rate_hz = 30;
bool left, center, right; 
int steeringAngle, auxMax = 0; 
float convolvedStates[7]; 
//Functions used in main
void publishControl(); //sets a new angular and linear speed to AutoNOMOS_mini_1 
void getLeftLine(std_msgs::Int32MultiArray);
void getCenterLine(std_msgs::Int32MultiArray);
void getRightLine(std_msgs::Int32MultiArray);
void getSteeringAngle(std_msgs::Int16);
void convolveStates(float x[], float y[]);


int main(int argc, char **argv){
	ros::init(argc, argv, "car_pose");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("Car pose node inicialized");

		left = false; 
		center = false; 
		right = false; 

	ros::Publisher states = nh.advertise <std_msgs::Float32MultiArray>("/states_node", rate_hz);
	//Subscribe to left, center and right line perceived from the rosbag 'rosbag_SDI11911'
	ros::Subscriber left_line = nh.subscribe("/points/left",1000, &getLeftLine);
	ros::Subscriber center_line = nh.subscribe("/points/center",1000, &getCenterLine);
	ros::Subscriber right_line = nh.subscribe("/points/right",1000, &getRightLine);
	ros::Subscriber steering_angle = nh.subscribe("/manual_control/steering",1000, &getSteeringAngle);

	ros::Publisher current_estimated_pose = nh.advertise <std_msgs::String>("/estimated_pose",rate_hz);

	ros::Rate rate(rate_hz);

	while(ros::ok()){
		float e1[7] = {0, 0.05, 0.3, 0.3, 0, 0.05, 0.3};
		float e2[7] = {0.05, .95, 0.0, 0.0, 0.0, 0.0, 0.0};
		float e3[7] = {0.0, 0.0, 0.0, 0.0, 0.05, 0.95, 0.0};
		float e4[7] = {0.05, 0.15, 0.8, 0.0, 0.0, 0.0, 0.0};
		float e5[7] = {0.0, 0.0, 0.0, 0.0, 0.05, 0.15, 0.8};

		std_msgs::Float32MultiArray a; //return array from convolved states

		 if(left && center && right){ //Estado 1
            current_state.data = "I'm right in the MIDDLE";
            if(steeringAngle > 0 && steeringAngle < 57){
            	convolveStates(e1,e1);
            }else if (steeringAngle < 114)
            	convolveStates(e1,e1);
            	else convolveStates(e1,e5);
            
            a.data.push_back(convolvedStates[0]);
            a.data.push_back(convolvedStates[1]);
            a.data.push_back(convolvedStates[2]);
            a.data.push_back(convolvedStates[3]);
            a.data.push_back(convolvedStates[4]);
            a.data.push_back(convolvedStates[5]);
            a.data.push_back(convolvedStates[6]);
            states.publish(a);
        }else if(left && !center && !right){ //Estado 2
        	current_state.data = "Izquierda carretera";
        	if(steeringAngle > 0 && steeringAngle < 57){
        		convolveStates(e2,e2);
        	}else if(steeringAngle < 100) convolveStates(e2,e4);
        		  else convolveStates(e2,e1);

        	a.data.push_back(convolvedStates[0]);
            a.data.push_back(convolvedStates[1]);
            a.data.push_back(convolvedStates[2]);
            a.data.push_back(convolvedStates[3]);
            a.data.push_back(convolvedStates[4]);
            a.data.push_back(convolvedStates[5]);
            a.data.push_back(convolvedStates[6]);
            states.publish(a);
		}else if(!left && !center && right){ //Estado 3
        	current_state.data = "Derecha carretera";
        	if(steeringAngle < 0) convolveStates(e3,e5);
        	else convolveStates(e3,e3);

        	a.data.push_back(convolvedStates[0]);
            a.data.push_back(convolvedStates[1]);
            a.data.push_back(convolvedStates[2]);
            a.data.push_back(convolvedStates[3]);
            a.data.push_back(convolvedStates[4]);
            a.data.push_back(convolvedStates[5]);
            a.data.push_back(convolvedStates[6]);      	
     		states.publish(a);
        }else if(left && center && !right){ //Estado 4
        	current_state.data = "Sobre la línea izquierda";

        	if(steeringAngle < 0) convolveStates(e4,e2);
        	else if(steeringAngle > 0 && steeringAngle < 57) convolveStates(e4,e4);
        		else convolveStates(e4,e1);

        	a.data.push_back(convolvedStates[0]);
            a.data.push_back(convolvedStates[1]);
            a.data.push_back(convolvedStates[2]);
            a.data.push_back(convolvedStates[3]);
            a.data.push_back(convolvedStates[4]);
            a.data.push_back(convolvedStates[5]);
            a.data.push_back(convolvedStates[6]);	
      		states.publish(a);
        }else if(!left && center && right){ //Estado 5
        	current_state.data = "Sobre la línea derecha";

        	if(steeringAngle < 0) convolveStates(e5,e1);
        	else if(steeringAngle > 0 && steeringAngle < 57) convolveStates(e5,e5);
        	     else convolveStates(e5,e3);

        	a.data.push_back(convolvedStates[0]);
            a.data.push_back(convolvedStates[1]);
            a.data.push_back(convolvedStates[2]);
            a.data.push_back(convolvedStates[3]);
            a.data.push_back(convolvedStates[4]);
            a.data.push_back(convolvedStates[5]);
            a.data.push_back(convolvedStates[6]);
            states.publish(a);          	
        }
		
		current_estimated_pose.publish(current_state);
		left = false; 
		center = false; 
		right = false; 

		ros::spinOnce();

		rate.sleep();
	}
return 0; 
}

void getLeftLine(std_msgs::Int32MultiArray l_line){
		left = true; 
}

void getCenterLine(std_msgs::Int32MultiArray l_line){
		center = true; 
}

void getRightLine(std_msgs::Int32MultiArray l_line){
		right = true; 
}

void getSteeringAngle(std_msgs::Int16 steerAngle){
		steeringAngle = steerAngle.data; 
}

void convolveStates(float x[], float y[]){
	/*int max = 7;
	for (int i= 0; i< 7; i++){
		aux[max-1] = y[i]; 
		max--;
	}*/
	convolvedStates[0]  = x[0]*y[0];
	convolvedStates[1]  = x[0]*y[1] + x[1]*y[0];
	convolvedStates[2]  = x[0]*y[2] + x[1]*y[1] + x[2]*y[0];
	convolvedStates[3]  = x[0]*y[3] + x[1]*y[2] + x[2]*y[1] + x[3]*y[0];
	convolvedStates[4]  = x[0]*y[4] + x[1]*y[3] + x[2]*y[2] + x[3]*y[1] + x[4]*y[0];
	convolvedStates[5]  = x[0]*y[5] + x[1]*y[4] + x[2]*y[3] + x[3]*y[2] + x[4]*y[1] + x[5]*y[0];
	convolvedStates[6]  = x[0]*y[6] + x[1]*y[5] + x[2]*y[4] + x[3]*y[3] + x[4]*y[2] + x[5]*y[1] + x[6]*y[0];
	
	convolvedStates[7]  = x[6]*y[1] + x[5]*y[2] + x[4]*y[3] + x[3]*y[4] + x[2]*y[5] + x[1]*y[6];
	convolvedStates[8]  = x[6]*y[2] + x[5]*y[3] + x[4]*y[4] + x[3]*y[5] + x[2]*y[6];
	convolvedStates[9]  = x[6]*y[3] + x[5]*y[4] + x[4]*y[5] + x[3]*y[6];
	convolvedStates[10] = x[6]*y[4] + x[5]*y[5] + x[4]*y[6];
	convolvedStates[11] = x[6]*y[5] + x[5]*y[6];
	convolvedStates[12] = x[6]*y[6];

	convolvedStates[0] = convolvedStates[12] + convolvedStates[0];
	convolvedStates[1] = convolvedStates[11] + convolvedStates[1];
	convolvedStates[2] = convolvedStates[10] + convolvedStates[2];
	convolvedStates[3] = convolvedStates[9] + convolvedStates[3];
	convolvedStates[4] = convolvedStates[8] + convolvedStates[4];
	convolvedStates[5] = convolvedStates[7] + convolvedStates[5];
	
}
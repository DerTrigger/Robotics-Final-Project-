#include <ros/ros.h>
#include <geometry_msgs/Twist.h> //guardar los valores de x, y, y theta
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include "gazebo_msgs/LinkStates.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"


float PI = 3.14159265; //define PI 

double rate_hz = 4;  //speed at which we make iterations
float lidar_lecutres[360];
float angles_vector[360];
float angle = 0, distance = 0; 
float xPrima= 0, yPrima= 0;

//Set linear and angular speeds with  "move to point" MODEL
float vel, gam; 
double kv = 3; //gain to set speed
float distanceToObstacle;
std_msgs::Float32 msgVel;
std_msgs::Float32 msgGamma; //messages that topics from autonomous car receive
//Data that estimates the pose of obstacle robot
float obsPose[2][2];

//Kalman Filter, first iteration
float lastP = 0.05, R = 0.1; //variance and noise in environment
float lastMeasure[2][2] = {0,0,0,0};

//Functions used during process
void getLidarSensor(sensor_msgs::LaserScan);
void fill_angles_vector();
void obstaclePoseFromOurCar();
void kalman_control();
void toVelocitiesConvertion();
void setSpeeds();
bool isObstacleFar();
//-----------------------------------------------------------------MAIN-----------------------------------------------------
int main(int argc, char **argv){
	ros::init(argc, argv, "kalman_control");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("Control car with Kalman filter initialized");

	ros::Publisher give_carSpeed = nh.advertise<std_msgs::Float32>("/AutoNOMOS_mini_1/manual_control/velocity", rate_hz);
	ros::Publisher give_carSteering = nh.advertise<std_msgs::Float32>("/AutoNOMOS_mini_1/manual_control/steering", rate_hz);

	ros::Subscriber lidar_mini_1 = nh.subscribe("/AutoNOMOS_mini_1/laser_scan", 1000, &getLidarSensor);
	ros::Rate rate(rate_hz);

	obsPose[1][0] = xPrima; 
	obsPose[1][1] = yPrima; 
	fill_angles_vector();


	while(ros::ok()){
		obstaclePoseFromOurCar(); //Obtain x and y
		kalman_control(); //Obtain estimated pose
		//ROS_INFO_STREAM("Obstacle measured (x,y):  (" << obsPose[0][0] << ", " << obsPose[0][1] << ")");

		toVelocitiesConvertion();
		setSpeeds();

		give_carSpeed.publish(msgVel);
		give_carSteering.publish(msgGamma);
		ros::spinOnce();
		rate.sleep();
	}

	return 0; 
}


//--------------------------------------------------------------------FUNCTIONS--------------------------------------------
void getLidarSensor(sensor_msgs::LaserScan lidar_from_mini){
	for (int i = 0; i< 360; i++){
		lidar_lecutres[i] = lidar_from_mini.ranges[i];
	}
}

void fill_angles_vector(){
	float pos = 0; 
	for(int i = 0; i< 360; i++){
		angles_vector[i] = pos; 
		pos+= 0.0175019223243; 
	}
}

void obstaclePoseFromOurCar(){
	for (int i = 0; i < 360; i++){
		if(lidar_lecutres[i]> .19 && lidar_lecutres[i] < 6){
			angle = angles_vector[i];
			distance = lidar_lecutres[i]; 
		}
	}

	obsPose[0][0] = distance*cos(angle*180/PI);//x
	obsPose[0][1] = distance*sin(angle*180/PI);//y

	ROS_INFO_STREAM("Posición medida con LIDAR: (" << obsPose[0][0] << ", " << obsPose[0][1] << ")");
	yPrima = (obsPose[0][1] - yPrima)/4;
	xPrima = (obsPose[0][0] - xPrima)/4;

	obsPose[1][0] = xPrima;//xprima
	obsPose[1][1] = yPrima;//yprima

}

/*
void calculate_speeds(){
	t0= ros::Time::now().toSec();
}*/

void kalman_control(){
	float actualMeasure[2][2]; //Xk

	//fill actualMeasure 
	for(int i = 0; i< 2; i++){
		for(int j = 0; j< 2; j++){
			actualMeasure[i][j] = lastMeasure[i][j];
		}
	}
	float measures[2][2];
	float actualP = lastP; 
	float gainKalman = actualP /(actualP + R);  
	measures[0][0] = obsPose[0][0]; //Position respect to x-axis
	measures[0][1] = obsPose[0][1]; //Position respect to y-axis
	measures[1][0] = obsPose[1][0]; //Velocity in x
	measures[1][1] = obsPose[1][1]; //Velocity in y
	
	float newX, newY, newYprima, newXprima, newP;

	newP = (1-gainKalman)*actualP; 
	newX = actualMeasure[0][0] + gainKalman*(measures[0][0] - actualMeasure[0][0]);
	newY = actualMeasure[0][1] + gainKalman*(measures[0][1] - actualMeasure[0][1]);
	newXprima = actualMeasure[1][0] + gainKalman*(measures[1][0] - actualMeasure[1][0]);
	newYprima = actualMeasure[1][1] + gainKalman*(measures[1][1] - actualMeasure[1][1]);
	ROS_INFO_STREAM("Estimated pose (x,y): (" << newX << ", " << newY << ")");
	lastMeasure[0][0] = newX;
	lastMeasure[0][1] = newY;
	lastMeasure[1][0] = newXprima;
	lastMeasure[1][1] = newYprima;
	lastP = newP; 
}

//CONTROL FOR Autonomos_mini_1 to FOLLOW the OBSTACLE ROBOT
void toVelocitiesConvertion(){
	
	distanceToObstacle = sqrt(pow(lastMeasure[0][0],2) + pow(lastMeasure[0][1],2));
	vel = kv*distanceToObstacle; 
	gam = atan(lastMeasure[0][1]/lastMeasure[0][0]);
}

void setSpeeds(){
	/*if(isObstacleFar()){
		msgVel.data = vel;
		msgGamma.data = gam;
	}else{
		msgVel.data = 0;
		msgGamma.data = 0;
	}*/
	msgVel.data = vel;
	msgGamma.data = gam;
}

bool isObstacleFar(){
	bool flag = false;
	if(distanceToObstacle > 0.5)
		flag = true;
return flag;
}

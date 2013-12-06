/*
 * Parking.cpp
 *
 *  Created on: Dec 6, 2013
 *  Author: Razi Ghassemi
 */

#include "Parking.h"
#include "ConstPark.h"




Parking::Parking() {
	// TODO Auto-generated constructor stub

}

Parking::~Parking() {
	// TODO Auto-generated destructor stub
}


void Parking::scanValues(const sensor_msgs::LaserScan laser) {



}

void Parking::init(){


	angle_pub = parkingNode.advertise<std_msgs::Int16>("angle_cmd", 1);
    speed_pub = parkingNode.advertise<std_msgs::Int8>("speed_cmd", 1);
    hokuyoSubscriber =parkingNode.subscribe("/scan", 1, &Parking::scanValues, this);
    ros::Duration(1).sleep();
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "collisionDetection");
	Parking park;
	try {
		park.init();
	} catch (std::exception& error) {
		ROS_ERROR("Error: %s\n", error.what());
		return -1;
	} catch (...) {
		ROS_ERROR("Unknown Error\n\r");
		return -1;
	}

	ros::Rate loop_rate(LOOP_RATE);

	while (ros::ok) {


        ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

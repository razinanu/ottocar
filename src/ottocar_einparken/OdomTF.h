/*
 * OdomTF.h
 *
 *  Created on: 05.05.2014
 *      Author: jsabsch
 */

#ifndef ODOMTF_H_
#define ODOMTF_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>

class OdomTF {
public:
	OdomTF();
	virtual ~OdomTF();
	void transform();

private:

	void odomCallback(nav_msgs::Odometry msg);
	void velCallback(std_msgs::Float32 msg);

	ros::Subscriber odomSub;
	ros::Subscriber velSub;
	tf::TransformBroadcaster odom_broadcaster;
	nav_msgs::Odometry lastMsg;
	float distance;
	float velocity;
};

#endif /* ODOMTF_H_ */

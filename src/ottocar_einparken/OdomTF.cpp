/*
 * OdomTF.cpp
 *
 *  Created on: 05.05.2014
 *      Author: jsabsch
 */

#include "OdomTF.h"

OdomTF::OdomTF()
{
	ros::NodeHandle nh;
	odomSub = nh.subscribe("odom", 1, &OdomTF::odomCallback, this);
	velSub = nh.subscribe("sensor_motor_revolution", 1, &OdomTF::velCallback, this);

	velocity = 0;
	distance = 0;
}

OdomTF::~OdomTF() {
	// TODO Auto-generated destructor stub
}

void OdomTF::odomCallback(nav_msgs::Odometry msg)
{
	lastMsg = msg;
}

void OdomTF::velCallback(std_msgs::Float32 msg)
{
	velocity = msg.data - distance;
	distance = msg.data;
}

void OdomTF::transform()
{
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.header.stamp=ros::Time::now();

	static int seq=0;

	odom_trans.header.seq=seq++;
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.x = velocity;//lastMsg.twist.twist.linear.x;
	odom_trans.transform.translation.y = 0.0;//lastMsg.twist.twist.linear.y;
	odom_trans.transform.translation.z = 0.0;//lastMsg.twist.twist.linear.z;
	odom_trans.transform.rotation = odom_quat; //lastMsg.pose.pose.orientation;


	//START
	 odom_trans.header.stamp=ros::Time::now();
	 odom_trans.header.seq=seq++;
	 odom_broadcaster.sendTransform(odom_trans);
	//ENDE

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odomTF");
	OdomTF odom;
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		odom.transform();

		ros::spinOnce();
		loop_rate.sleep();
	}
}

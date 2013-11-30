/*
 * LaseScan.cpp
 *
 *  Created on: Nov 28, 2013
 *  Author: Razi Ghassemi
 */

#include "LaserScan.h"

laserState currentSearchState = start;

void scanValues(const sensor_msgs::LaserScan laser) {

	minVDistance = 0;
	angle = laser.angle_max;
	//just as first program, to scan just one time!
       if (c == 0) {

		for (unsigned int i = laser.ranges.size() - 1;
				i > (laser.ranges.size()) / 2; i--) {

			if (currentSearchState == start && laser.ranges[i] < 0.3)

			{

				currentSearchState = calculateBaseRange;
				ROS_INFO(
						"Start value: angle=[%f],range[%d]=[%f]: ", (angle * 180) / Pi, i, laser.ranges[i]);
			}
			if (currentSearchState
					== calculateBaseRange&& laser.ranges[i]!=0.0&& laser.ranges[i] != INFINITY) {
			//todo if another direction, it should be fixed

if			(2 *laser.ranges[i] < laser.ranges[i - 1]&& laser.ranges[i-1]!=0.0&& laser.ranges[i-1] != INFINITY) {
				currentSearchState = calculateMaxPoint;

				baseVDistance = laser.ranges[i]
				* sin(laser.angle_max - angle);
				baseHDistance = laser.ranges[i]
				* cos(laser.angle_max - angle);
				ROS_INFO("BaseRange value: angle=[%f],range[%d]=[%f]:  ", (angle * 180) / Pi, i, laser.ranges[i]);
				ROS_INFO("Next point after BaseRange value: angle=[%f],range[%d]=[%f]:  ", (angle * 180) / Pi, i-1, laser.ranges[i-1]);
				ROS_INFO("BaseRange Horizontal:%f Vertical:%f   ",baseHDistance,baseVDistance);
			}
		}

			//todo if another direction, it should be fixed
			if (currentSearchState
					== calculateMaxPoint&&laser.ranges[i] > (laser.ranges[i -1])
					&& laser.ranges[i-1]!=0.0&& laser.ranges[i-1] != INFINITY) {

currentSearchState			= calculateMinPoint;
		}

			if (currentSearchState
					== calculateMinPoint&& laser.ranges[i] <(laser.ranges[i -1])
					&& laser.ranges[i-1]!=0.0&& laser.ranges[i-1] != INFINITY) {

secondHDistance			=laser.ranges[i]* cos(laser.angle_max - angle);
			ROS_INFO("Second Point: angle=[%f],range[%d]=[%f]:  ", (angle * 180) / Pi, i, laser.ranges[i]);
			ROS_INFO("Second Horizontal:%f ",secondHDistance);
			if(abs(secondHDistance-baseHDistance)<MINDISTANCE) {

				minVDistance=laser.ranges[i]
				* sin(laser.angle_max - angle);
				ROS_INFO("MIN value: angle=[%f],range[%d]=[%f]:  ", (angle * 180) / Pi, i, laser.ranges[i]);
				ROS_INFO("MIN Vertical: %f   ",minVDistance);
				currentSearchState = end;
			}

		}

			angle -= laser.angle_increment;

		}
		c = 1;
	}
	if (minVDistance != 0) {
		space = minVDistance - baseVDistance;
		ROS_INFO("Space is: %f ", space);
	}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "hLaserReader");
	ros::NodeHandle n;
	ros::Subscriber hokuyoSubscriber = n.subscribe("/scan", 1, scanValues);
	ros::spin();
	return 0;
}


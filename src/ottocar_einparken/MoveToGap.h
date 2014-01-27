/*
 * DriveToGap.h
 *
 *  Created on: 06.12.2013
 *      Author: licht
 */

#ifndef MOVETOGAP_H_
#define MOVETOGAP_H_

#include "ConstPark.h"
#include "ParallelController.h"
#include "Parking.h"

class MoveToGap
{
public:
	struct driveData
	{
		std_msgs::Int8 speed;
		std_msgs::Int8 angle;
	};

	MoveToGap();
	virtual ~MoveToGap();

	driveData moveToGap(float dataIRside, float dataIRback,  float distanceToGap, float voltage);
private:
	bool cartonSeen;
	bool distanceGot;
	ros::Time gapBegin;
	ros::Time distanceFound;
	ros::Time lastTime;
	float timeToDrive;
	int mode;

	float calculateSpeed10(float voltage);
};

#endif /* MOVETOGAP_H_ */

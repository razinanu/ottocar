/*
 * ParkingController.cpp
 *
 *  Created on: Dec 7, 2013
 *      Author: Razi Ghassemi
 */

#include "ParkingController.h"

ParkingController::ParkingController()
{
	minimalDistance = -1;
}

void ParkingController::LaserScanParkControll(const sensor_msgs::LaserScan laser)
{
	laserData.clear();

	for(int i=0; i<laser.ranges.size(); i++)
	{
		if(isnan(laser.ranges[i]) )
		{
			laserData.push_back(-1);
		}
		else
		{
			laserData.push_back(laser.ranges[i]);
		}
	}

	minimalDistance = -1;
}

ParkingController::~ParkingController()
{

}

float ParkingController::getBackDistance()
{
	return 0;
}

float ParkingController::getMinimalDistance()
{
	if(minimalDistance != -1)
	{
		for(int laserIndex=0; laserIndex < laserData.size(); laserIndex++)
		{
			if(minimalDistance > laserData[laserIndex] || minimalDistance == -1)
			{
				minimalDistance = laserData[laserIndex];
			}
		}
	}

	return minimalDistance;
}

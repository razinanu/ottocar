/*
 * ParkingController.cpp
 *
 *  Created on: Dec 7, 2013
 *      Author: Simone Bexten
 */

#include "ParkingController.h"

ParkingController::ParkingController() :
		RIGHTTURN(0.18), LEFTTURN(0.28), TOFRONT(0.13)
{
	right_turn = true;
	left_turn = false;
	straight_turn = false;
	minimalDistance = -1;
	verticalDistance = 0;
	horizontalDistance = 0;
}

ParkingController::~ParkingController()
{

}

bool ParkingController::stopTurn()
{
	return straight_turn;
}

bool ParkingController::rightTurn()
{
	return right_turn; // false -> leftturn
}

int ParkingController::findEdge()
{
	int indexEdge = 0;
	float min = -1;

	for (int i = 0; i < this->laser.ranges.size(); i++)
	{
		if (min > laser.ranges[i] || min == -1)
		{
			min = laser.ranges[i];
			indexEdge = i;
		}
	}

//	float angle = laser.angle_min;
//
//	float HDistance = 0.0;
//	float baseHDistance = 0.0;
//
//	for (int i = laser.ranges.size() - 1; i > (laser.ranges.size()) / 2; i--)
//	{
//
//		float ray = laser.ranges[i];
//		float rayAngle = laser.angle_max - angle;
//
//		HDistance = laser.ranges[i] * cos(rayAngle);
//		if (HDistance < PARALLELDISTANCE) //kleiner als 13cm
//		{
//			ROS_INFO("Start value: angle=[%f],range[%d]=[%f]: ",
//					(angle * 180) / M_PI, i, laser.ranges[i]);
//		} //endif
//
//		if (2 * laser.ranges[i] < laser.ranges[i - 1])
//		{
////				baseVDistance = laser.ranges[i] * sin(laser.angle_max - angle);
//			baseHDistance = laser.ranges[i] * cos(laser.angle_max - angle);
////				ROS_INFO("BaseRange value: angle=[%f],range[%d]=[%f]: ",
////				ROS_INFO("[Parking]:");
////			}
////
//			if (laser.ranges[i] < laser.ranges[i - 1]
//					&& laser.ranges[i] < laser.ranges[i + 1])
//			{
//				indexEdge = i;
//			} // endif
//		} // endif
//		angle -= laser.angle_increment;
//	} // endfor
	return indexEdge;
}

void ParkingController::turnDistance(const sensor_msgs::LaserScan laser)
{
	float angle = laser.angle_min;
	int indexEdge = this->findEdge();

// nur die rechte Seite betrachten
//	ROS_INFO("ParkingController angle: %f", (angle * 180) / M_PI);
	float rayAtMinEdge = laser.ranges[indexEdge];
	ROS_INFO("index: %i| winkel: %f", indexEdge, laser.angle_increment);
	float rayAccordingToEdge = laser.ranges[indexEdge - 21];

	ROS_WARN("(1./2.), (%f,%f)", rayAtMinEdge, rayAccordingToEdge);

	if (rayAccordingToEdge > rayAtMinEdge * 2)
	{
		return;
	}

	float alpha = laser.angle_increment * 20;
	///##################################### test################################
	float gamma = laser.angle_increment * 20;
	float c = sqrt(
			(rayAtMinEdge * rayAtMinEdge) + (rayAccordingToEdge * rayAccordingToEdge) -
			(2 * rayAtMinEdge * rayAccordingToEdge) * cos(gamma));
	ROS_INFO("c: %f", c); //ok

	float alphaOnObstacle = acos(((rayAtMinEdge*rayAtMinEdge)
			+(c*c)-(rayAccordingToEdge*rayAccordingToEdge))
					/(2*rayAtMinEdge * c));
	ROS_INFO("alphaOnObstacle: %f", alphaOnObstacle);
	ROS_INFO("---Triangle Complement - cannot see ---");
	float alphaComplement = M_PI - alphaOnObstacle; // 180° = M_PI
	// falls der Winkel gamma benoetigt
	// gamma => ziwschen horizont und erstem Strahl
	float gammaComplement = M_PI - (M_PI / 2) - alphaComplement; // 180 - 90 -alphaComplement

	float horizontalDistanceToObstacle = sin(alphaComplement) * rayAtMinEdge;
	float verticalDistanceToObstacle = cos(alphaComplement) * rayAtMinEdge;
	ROS_INFO("[PAC]: new is (h,v) - (%f, %f)",
			horizontalDistanceToObstacle, verticalDistanceToObstacle);

	/////##################################test ende################################
	float beta = asin((rayAtMinEdge * sin(alpha)) / rayAccordingToEdge);
	float gammaC = M_PI - alpha - beta;

	float epsilon = (M_PI / 2) - M_PI - gammaC;

//	ROS_INFO("ParkingController epsilon: %f", (epsilon * 180) / M_PI);
	horizontalDistance = sin(epsilon) * rayAtMinEdge;
	verticalDistance = sin(epsilon + alpha) * rayAccordingToEdge;
	ROS_INFO("[PaCo]: distance is (h,v) - (%f, %f)",
					horizontalDistance, verticalDistance);
}

void ParkingController::LaserScanParkControll(
		const sensor_msgs::LaserScan laser)
{
	this->laser = laser;
	minimalDistance = -1;

	this->turnDistance(laser);

	// Rechtskurve
	if (right_turn && horizontalDistance < RIGHTTURN
			&& horizontalDistance > (PARALLELDISTANCE))
	{
		ROS_INFO("[PaCo]: right - distanceH is %f", horizontalDistance);
		right_turn = true;
	}
	else
	{
		right_turn = false; // Linkskurve beginnen
		left_turn = true;
	}

	// Linkskurve
	if (left_turn && verticalDistance < LEFTTURN)
	{
		ROS_INFO("[PaCo]: left- distanceV is %f", verticalDistance);
	}
	else
	{
		left_turn = false;
		straight_turn = true;
	}

	if (straight_turn && verticalDistance < SECUREDISTANCE_FRONT
			&& horizontalDistance < MINDISTANCE)
	{
		ROS_INFO("[PaCo]: straight distance is (h,v) - (%f, %f)",
				horizontalDistance, verticalDistance);
	}
	else
	{
		straight_turn = false;
	}

	// Parken nachziehen

	// rechts old
//	if (horizontalDistance < RIGHTTURN || horizontalDistance < PARALLELDISTANCE)
//	else
//	{
//
//		if (verticalDistance < LEFTTURN)
//		{
//			right_turn = false;
//			ROS_WARN("ParkingController: left");
//		}
//		else
//		{
//			straight_turn = true;
//		}
//	}
}

float ParkingController::getBackDistance()
{
	return 0;
}

float ParkingController::getMinimalDistance()
{
// aktualisieren, da -1 schlecht beim vergleichen der werte besser 99 (BIGRANGE)
	if (minimalDistance == -1)
	{
		for (int i = 0; i < this->laser.ranges.size(); i++)
		{
			if (minimalDistance > laser.ranges[i] || minimalDistance == -1)
			{
				minimalDistance = laser.ranges[i];
			}
		}
	}

	return minimalDistance;
}

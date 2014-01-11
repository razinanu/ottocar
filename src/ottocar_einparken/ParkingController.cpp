/*
 * ParkingController.cpp
 *
 *  Created on: Dec 7, 2013
 *      Author: Simone Bexten
 *      export ROS_MASTER_URI=http://ottocar.local:11311/
 *
 */

#include "ParkingController.h"

ParkingController::ParkingController() :
		RIGHTTURN(0.18), LEFTTURN(0.28), TOFRONT(0.13)
{
	right_turn = true;
	left_turn = false;
	straight_turn = false;
	minimalDistance = 999;
	verticalDistanceToObstacle = 0;
	horizontalDistanceToObstacle = 0;
	rearWheelHorizontal = -1;
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

int ParkingController::findMinEdge(int indexMaxEdge)
{
	int indexEdge = indexMaxEdge;
	float min = -1;

	for (int i = indexMaxEdge; i < this->laser.ranges.size(); i++)
	{
		if (min > laser.ranges[i] || min == -1)
		{
			min = laser.ranges[i];
			indexEdge = i;
		}
	}
	return indexEdge;
}

int ParkingController::findMaxForHorizontal()
{
	int indexEdge = this->laser.ranges.size() / 2;

	// von links laufen
	// nur die haelfte des arrays betrachten
	for (int i = this->laser.ranges.size() / 2; i < this->laser.ranges.size();
			i++)
	{
		// nur mit werten vergleichen, die unter 1.0 meter liegen
		// TODO werte abschaffen?
		if (laser.ranges[i] < 1.0)
		{
			// Sprung und damit auessere Ecke gefunden
			// wenn dieser scan weniger als halb so klein wie der vorherige ist
			if (laser.ranges[i] < (laser.ranges[i - 1] / 2))
			{
				//ROS_INFO("C");
				indexEdge = i;
			}
		}
	}
	return indexEdge;
}

int ParkingController::findMaxForVertical()
{
	int indexEdge = 0;

	// von rechts laufen
	for (int i = laser.ranges.size() - 1; i > 0; i--)
	{
		// sprung und damit auessere ecke gefunden
		if (laser.ranges[i] < 2 * laser.ranges[i - 1])
		{
			indexEdge = i;
			return indexEdge;

		}
	}
	return indexEdge;
}

void ParkingController::turnDistance(const sensor_msgs::LaserScan laser)
{
	float angle = laser.angle_min;
	int indexMaxEdge = this->findMaxForHorizontal();
	int indexMinEdge = this->findMinEdge(indexMaxEdge);

	float rayAtMinEdge = laser.ranges[indexMinEdge];
	float rayAccordingToEdge = laser.ranges[indexMaxEdge];
	float gamma = laser.angle_increment * (abs(indexMinEdge) - abs(indexMaxEdge));

	ROS_WARN("[PAC]: (minEdge,maxEdge)-(%i,%i)-(%f,%f): %2.8f", indexMinEdge,
			indexMaxEdge, rayAtMinEdge, rayAccordingToEdge, gamma);

////	if (rayAccordingToEdge > rayAtMinEdge * 2)
////	{
////		return;
////	}
//	// TODO: rechung beim schraegstehen testen --> ist falsch
////	[ WARN] [1387057245.476250513]: [PAC]: (minEdge,maxEdge)-(511,478)-(0.177000,0.218000): 0.00025157
////	[ INFO] [1387057245.476900005]: [PAC]: sideC of Triangle: 0.041000
////	[ INFO] [1387057245.477247833]: [PAC]: ---Triangle Complement - cannot see ---
////	[ INFO] [1387057245.477533480]: [PAC]: distances(h,v) - (0.000293, 0.177000)
////	[ INFO] [1387057245.477710817]: [PAC]: left - vertical Distance is 0.177000
//
//	// sideC is on the obstacle
//	// c = sqrt(a²+b²-2ab*cos(gamma));

	float sideC = sqrt(
			(rayAccordingToEdge * rayAccordingToEdge)
				+ (rayAtMinEdge * rayAtMinEdge)
				- (2 * rayAtMinEdge * rayAccordingToEdge) * cos(gamma));

//	ROS_INFO("[PAC]: sideC of Triangle: %f", sideC);
// 	alpha = acos((b² + c² - a²) / 2*b*c);

	float alphaOnObstacle = acos(
			((rayAtMinEdge * rayAtMinEdge)
					+ (sideC * sideC)
					- (rayAccordingToEdge * rayAccordingToEdge))
					/ (2 * rayAtMinEdge * sideC));

	float epsilon = M_PI - gamma - alphaOnObstacle;
	//ROS_INFO("[PAC]: ---Triangle Complement - cannot see ---");
	float alphaComplement = M_PI - alphaOnObstacle; // 180° = M_PI

	ROS_INFO("epsilon: %f, alphaOnObstacle: %f, gamma: %f", ((360*epsilon)/(2*M_PI)),((360*alphaOnObstacle)/(2*M_PI)), ((360*gamma)/(2*M_PI)));
	//ROS_INFO("rayAccordingTo: %f, alphaOnObstacle: %f, gamma: %f", epsilon, alphaOnObstacle, gamma);

	horizontalDistanceToObstacle = sin(epsilon) * rayAccordingToEdge;
	verticalDistanceToObstacle = cos(alphaComplement) * rayAtMinEdge;
	ROS_INFO("[PAC]: distances(h,v) - (%f, %f)", horizontalDistanceToObstacle,
			verticalDistanceToObstacle);

//	float orientation1 = abs(indexMaxEdge) * laser.angle_increment;	//Winkel bis maxEdge
//	float orientation2 = M_PI / 2 - epsilon;						//Winkel zwischen MaxEdge und horizontal
//	float orientation = M_PI - (orientation1 + orientation2);
	float orientation = (M_PI / 2) - (abs(indexMaxEdge) * laser.angle_increment) + epsilon;
	rearWheelHorizontal = this->calculateRearWheelHorizontal(horizontalDistanceToObstacle, orientation);
}

float ParkingController::calculateRearWheelHorizontal(float frontHorizontal, float orientation)
{
	float LASER_REAR_DISTANCE = 0.24;

	//cos(a) * Hypothenuse = Ankathete
	float cathetus = cos(orientation) * LASER_REAR_DISTANCE;

	ROS_INFO("[PAC::calculateRearWheelHorizontal] cathetus: %f", cathetus);
	ROS_INFO("[PAC::calculateRearWheelHorizontal] return %f", (frontHorizontal - cathetus));

	return frontHorizontal - cathetus;
}

void ParkingController::LaserScanParkControll(const sensor_msgs::LaserScan laser)
{
	this->laser = laser;
	minimalDistance = -1;
	rearWheelHorizontal = 999;

	this->turnDistance(laser);
	//if (horizontalDistanceToObstacle < RIGHTTURN)
	if(rearWheelHorizontal < 0)
	{
		right_turn = false;
		//left_turn = false;
	}
//	else if (verticalDistanceToObstacle < LEFTTURN)
//	{
//		ROS_INFO("left");
//		//left_turn = true;
//		right_turn = false;
//	}
	else
	{
		//right_turn = true;
	}

	// Rechtskurve
//	if (right_turn && horizontalDistanceToObstacle < RIGHTTURN)
//			//&& horizontalDistanceToObstacle > PARALLELDISTANCE)
//	{
//		ROS_INFO("[PAC]: right - horizontal Distance is %f",
//				horizontalDistanceToObstacle);
//		right_turn = true;
//	}
//	else
//	{
//		right_turn = false; // Linkskurve beginnen
//		left_turn = true;
//	}
//
//	// Linkskurve
//	if (left_turn && verticalDistanceToObstacle < LEFTTURN)
//	{
//		ROS_INFO("[PAC]: left - vertical Distance is %f",
//				verticalDistanceToObstacle);
//	}
//	else
//	{
//		left_turn = false;
//		straight_turn = true;
//	}
//
//	if (straight_turn && verticalDistanceToObstacle < SECUREDISTANCE_FRONT
//			&& horizontalDistanceToObstacle < MINDISTANCE)
//	{
//		ROS_INFO("[PaCo]: straight distance is (h,v) - (%f, %f)",
//				horizontalDistanceToObstacle, verticalDistanceToObstacle);
//	}
//	else
//	{
//		straight_turn = false;
//	}

	// TODO Parken nachziehen
}

float ParkingController::getFrontDistance()
{
	return 0;
}

float ParkingController::getMinimalDistance()
{
// TODO  aktualisieren, da -1 schlecht beim vergleichen der werte besser 99 (BIGRANGE)
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

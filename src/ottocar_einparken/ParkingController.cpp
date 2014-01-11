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
	minimalDistance = -1;
	verticalDistanceToObstacle = 0;
	horizontalDistanceToObstacle = 0;
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
	int indexEdge = 0;
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
	int indexEdge = 0;

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
			// wenn dieser scan ist weniger als halb so klein wie der vorherige
			if (laser.ranges[i] < (laser.ranges[i - 1] / 2))
			{
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
		}
	}
	return indexEdge;
}

float ParkingController::computeTriangulationForDistance(float rayAtMinEdge,
		float rayAccordingToEdge, float angleGamma)
{
	// side is on the obstacle
	// c = sqrt(a²+b²-2ab*cos(gamma));
	float sideOnObstacle = sqrt(
			(rayAtMinEdge * rayAtMinEdge)
					+ (rayAccordingToEdge * rayAccordingToEdge)
					- (2 * rayAtMinEdge * rayAccordingToEdge)
							* cos(angleGamma));
//	ROS_INFO("[PAC]: sideOnObstacle of Triangle: %f", sideOnObstacle);

	// alpha = acos(b² + c² -a² / 2*b*c);
	float alphaOnObstacle = acos(
			((rayAtMinEdge * rayAtMinEdge) + (sideOnObstacle * sideOnObstacle)
					- (rayAccordingToEdge * rayAccordingToEdge))
					/ (2 * rayAtMinEdge * sideOnObstacle));

	ROS_WARN("[PAC]: ---Triangle Complement - cannot see ---");
	float alphaComplement = M_PI - alphaOnObstacle; // 180° = M_PI
	// falls der Winkel gamma benoetigt
	// gamma => zwischen horizont und erstem Strahl
	float gammaComplement = M_PI - (M_PI / 2) - alphaComplement; // 180 - 90 -alphaComplement

	return alphaComplement;
}

// erste haelfte des parkvorgangs berechnen
void ParkingController::turnOver()
{

	float angle = laser.angle_min;
	int indexMaxEdge = this->findMaxForHorizontal();
	int indexMinEdge = this->findMinEdge(indexMaxEdge);

	float rayAtMinEdge = laser.ranges[indexMinEdge];
	float rayAccordingToEdge = laser.ranges[indexMaxEdge];
	float angleGamma = laser.angle_increment * (indexMaxEdge - indexMinEdge);
//	ROS_WARN("[PAC]: (minEdge,maxEdge)-(%i,%i)-(%f,%f): %2.8f", indexMinEdge,
//			indexMaxEdge, rayAtMinEdge, rayAccordingToEdge, angleGamma);

	float alphaComplement = this->computeTriangulationForDistance(rayAtMinEdge,
			rayAccordingToEdge, angleGamma);

	horizontalDistanceToObstacle = sin(alphaComplement) * rayAtMinEdge;
	verticalDistanceToObstacle = cos(alphaComplement) * rayAtMinEdge;
	ROS_INFO("[PAC]: distances(h,v) - (%f, %f)", horizontalDistanceToObstacle,
			verticalDistanceToObstacle);
}

void ParkingController::LaserScanParkControll(
		const sensor_msgs::LaserScan laser)
{
	this->laser = laser;
	minimalDistance = -1;
	this->turnOver();

	if (horizontalDistanceToObstacle < RIGHTTURN)
	{
		right_turn = false;
	}

	if (straight_turn && verticalDistanceToObstacle < SECUREDISTANCE_FRONT
			&& horizontalDistanceToObstacle < MINDISTANCE)
	{
		ROS_INFO("[PaCo]: straight distance is (h,v) - (%f, %f)",
				horizontalDistanceToObstacle, verticalDistanceToObstacle);
	}

}

float computeParking(float rayEdge, float rayAccordingToEdge, float angle)
{
	// side is on the obstacle
	// c = sqrt(a²+b²-2ab*cos(gamma));
	float sideOnObstacle = sqrt(
			(rayEdge * rayEdge) + (rayAccordingToEdge * rayAccordingToEdge)
					- (2 * rayEdge * rayAccordingToEdge) * cos(angle));

	// alpha = acos(a² + b² -c² / 2*a*b);
	float angleOnObstacle = acos(
			((sideOnObstacle * sideOnObstacle) + (rayEdge * rayEdge)
					- (rayAccordingToEdge * rayAccordingToEdge))
					/ (2 * sideOnObstacle * rayEdge));

	ROS_WARN("[PAC]: ---Triangle Complement - cannot see ---");
	float angleBeta = M_PI - angle - angleOnObstacle; // 180° = M_PI
	float distanceToFront = sin(angleBeta) * rayAccordingToEdge;
	float distanceToStreet = cos(angleOnObstacle) * rayEdge;
	ROS_INFO("[PAC]: distance to street", distanceToStreet);

	return distanceToStreet;
}

float ParkingController::getDistanceToStreet()
{
	float distanceToStreet = 0.0;

	float angle = this->laser.angle_min;
	int indexMaxEdge = this->findMaxForVertical();

	float rayEdge = laser.ranges[indexMaxEdge];
	float rayAccordingToEdge = getMinimalDistance();
	float angleGamma = laser.angle_increment * (rayAccordingToEdge - rayEdge);

	ROS_INFO("[PAC]: --- Compute distance to street ---");
	ROS_WARN("[PAC]: %i-(%f,%f): %2.8f", indexMaxEdge, rayEdge,
			rayAccordingToEdge, angleGamma);

	distanceToStreet = this->computeTriangulationForDistance(rayEdge,
			rayAccordingToEdge, angleGamma);

	ROS_INFO("[PAC]: distances is %f", distanceToStreet);
	return distanceToStreet;
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

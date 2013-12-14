/*
 * ParallelController.cpp
 *
 *  Created on: Dec 6, 2013
 *      Author: licht
 */

#include "ParallelController.h"

float angle_old = 0;
int counter_punkt1 = -1;
int counter_punkt2 = -1;
//todo Puffer richtig vorfuellen
float buffer_D[] =
{ 0.15, 0.15, 0.15 };
float buffer_epsilon[] =
{ 0, 0, 0 };
int bufferSize = 3;
int bufferPointer = 0;

ParallelController::ParallelController()
{

}

ParallelController::~ParallelController()
{

}

void ParallelController::laserScanParallel(const sensor_msgs::LaserScan laser)
{
	//ersten Sprung finden

	//lokales Minimum/Ecke finden

	//solange a*k <= f  weiteren Sprung finden

	//d und epsilon bestimmen

	//Korrektur berechnen

	/*
	 * - a braucht eine Mindestgroesse
	 * - mehrere Dreiecke zwischen den beiden Punkten aufspannen
	 *   und Mittelwerte berechnen
	 * - Hindernis am Ende der Strecke betrachten
	 */

	// -------------------------------------------------------------------------
	int point_B;
	int point_C;

	int index = 0;

	triangleData triangle;

//		for (int i = 511; i > (256); i--)
//		{
//				ROS_INFO("i: %d range: %2.16f", i, laser.ranges[i]);
//		}

	//todo Minimumsfunktion verbessern/toleranter

	//pruefen, ob Bot neben dem Karton steht
	if (laser.ranges[laser.ranges.size() - (index + 1)] < TARGET_DISTANCE * TOLERANCE)
	{
		//ersten Sprung finden
		index = findEdge(laser, index);

		if (index >= SEARCH_SPACE)
		{
			ROS_ERROR("[PLC]: Optimierungsbedarf gefunden :)");
			return;
		}

		point_C = index;

		//lokales Minimum finden
		index = findMinimum(laser, index);

		if (index <= 0)
		{
			ROS_ERROR("[PLC]: das sollte noch verbessert werden :)");
			return;
		}

		point_B = index;
	}
	else //Bot steht NICHT neben dem Karton
	{
		//ersten gueltigen Wert finden
		index = findCarton(laser, index);

		if (index < 0)
		{
			ROS_ERROR("[PLC]: hier lief etwas nicht richtig :)");
			return;
		}

		//ersten Sprung finden
		index = findEdge(laser, index);
		if (index < 0)
		{
			ROS_ERROR("[PLC]: suboptimale Bedingungen ;)");
			return;
		}

		point_C = index;

		//lokales Minimum finden
		index = findMinimum(laser, index);

		if (index <= 0)
		{
			ROS_ERROR("[PLC]: kein tolles Minimum gefunde :(");
			return;
		}

		point_B = index;
	}

	triangle = calculateTriangle(laser, point_B, point_C);

	//Bedingung a*k <= f pruefen
	while ((triangle.side_a * CONFIDENCE < triangle.side_f) || (triangle.side_a < 0.1)
			|| (triangle.side_b == triangle.side_c)) //todo magischer Wert
	{
		index = point_C;

		//naechsten gueltigen Wert finden
		index = findCarton(laser, index);

		if (index < 0)
		{
			ROS_ERROR("[PLC]: da fehlen wohl Kartons ;)");
			return; //todo sollte hier nicht abbrechen
		}

		//naechsten Sprung finden
		index = findEdge(laser, index);

		if (index < 0)
		{
			ROS_ERROR("[PLC]: der Karton hat wohl keine Ecke ;)");
			return; //todo sollte hier nicht abbrechen
		}

		point_C = index;

		triangle = calculateTriangle(laser, point_B, point_C);
		writeToBuffer(triangle.side_d, triangle.epsilon);
	}
#if INFO == true
	ROS_INFO("[PLC]: point_b: %d", point_B);
	ROS_INFO("[PLC]: point_c: %d", point_C);
	ROS_INFO("[PLC]: side_d: %2.8f", triangle.side_d);
	ROS_INFO("[PLC]: side_d Mittelwert: %2.8f", getAverage().distance);
	ROS_WARN("[PLC]: side_d Median: %2.8f", getMedian().distance);
	ROS_INFO("[PLC]: epsilon Mittelwert: %2.8f", getAverage().angle;
	ROS_WARN("[PLC]: epsilon Median: %2.8f", getMedian().angle);
	ROS_INFO("[PLC]: -------------------------------------- ");
#endif
}

ParallelController::triangleData ParallelController::calculateTriangle(
		const sensor_msgs::LaserScan &laser, int point_B, int point_C)
{
	triangleData result;

	result.side_c = laser.ranges[laser.ranges.size() - (point_B + 1)];
	result.side_b = laser.ranges[laser.ranges.size() - (point_C + 1)];

	/* a = sqrt(b² + c² -2bc * cos alpha) */
	result.side_a =
			sqrt(
					(pow(result.side_b, 2) + pow(result.side_c, 2))
							- (2 * result.side_b * result.side_c
									* cos(
											(point_C - point_B)
													* laser.angle_increment)));

	/* beta = acos((b²-a²-c²)/-2ac) */
	result.beta = acos(
			(pow(result.side_b, 2) - pow(result.side_a, 2)
					- pow(result.side_c, 2))
					/ ((-2) * result.side_a * result.side_c));

	/* lambda = 180°-beta */
	result.lambda = M_PI - result.beta;

	/* f = cos lambda * c */
	result.side_f = cos(result.lambda) * result.side_c;

	//d berechnen
	result.side_d = result.side_c / sin(M_PI / 2) * sin(result.lambda);

	/* µ berechnen */
	result.mue = (M_PI / 2) - result.lambda;

	/* epsilon berechnen */
	result.epsilon = (point_B * laser.angle_increment) - result.mue;

#if INFO == true
	ROS_INFO("[PLC]: side_c: %2.8f", result.side_c);
	ROS_INFO("[PLC]: side_b: %2.8f", result.side_b);
	ROS_INFO("[PLC]: side_a: %2.8f", result.side_a);
	ROS_INFO(
			"[PLC]: alpha: %2.8f", ((point_C - point_B) * laser.angle_increment));
	ROS_INFO("[PLC]: beta: %2.8f", result.beta);
	ROS_INFO("[PLC]: lambda: %2.8f", result.lambda);
	ROS_INFO("[PLC]: side_f: %2.8f", result.side_f);
	ROS_INFO("[PLC]: side_d: %2.8f", result.side_d);
	ROS_INFO("[PLC]: mue: %2.8f", result.mue);
	ROS_INFO("[PLC]: epsilon: %2.8f", result.epsilon);
#endif

	return result;
}

int ParallelController::findEdge(const sensor_msgs::LaserScan &laser, int index)
{
	if (index < 0)
	{
		ROS_ERROR("[PLC]: findEdge falsch aufgerufen");
		return -1;
	}

	while ((laser.ranges[laser.ranges.size() - (index + 2)]
			< laser.ranges[laser.ranges.size() - (index + 1)] * TOLERANCE)
			&& (index < SEARCH_SPACE))
	{
		index++;
	}

	if (index >= SEARCH_SPACE)
	{
		return -1;
	}

	return index;
}

int ParallelController::findCarton(const sensor_msgs::LaserScan &laser,
		int index)
{
	if (index < 0)
	{
		ROS_ERROR("[PLC]: findCarton falsch aufgerufen");
		return -1;
	}

	do
	{
		index++;
	} while ((laser.ranges[laser.ranges.size() - (index + 1)]
			> (TARGET_DISTANCE / cos(index * laser.angle_increment)) * TOLERANCE)
			&& (laser.ranges[laser.ranges.size() - (index + 1)] > 3.0) //todo sensor magisch begrenzen
			&& (index < SEARCH_SPACE));

	if (index >= SEARCH_SPACE)
	{
		return -1;
	}

	return index;
}

//todo verbessern
int ParallelController::findMinimum(const sensor_msgs::LaserScan &laser,
		int index)
{
	if (index <= 0)
	{
		ROS_ERROR("[PLC]: findMinimum falsch aufgerufen");
		return -1;
	}

	do
	{
		index--;
	} while (((laser.ranges[laser.ranges.size() - (index + 1)]
			>= laser.ranges[laser.ranges.size() - (index + 0)])
	/*	|| (laser.ranges[laser.ranges.size() - (index + 1)]
	 >= laser.ranges[laser.ranges.size() - (index - 1)])
	 || (laser.ranges[laser.ranges.size() - (index + 1)]
	 >= laser.ranges[laser.ranges.size() - (index - 2)])) */) && (index > 2));

	if (index <= 0)
	{
		return -1;
	}

	return index;
}

void ParallelController::writeToBuffer(float value_D, float value_epsilon)
{
	buffer_D[bufferPointer] = value_D;
	buffer_epsilon[bufferPointer] = value_epsilon;
	bufferPointer++;

	if (bufferPointer >= bufferSize)
	{
		bufferPointer = 0;
	}
}

ParallelController::orientationData ParallelController::getAverage()
{
	orientationData result;
	result.angle = 0;
	result.distance = 0;

	for (int i = 0; i < bufferSize; i++)
	{
		result.distance += buffer_D[i];
		result.angle += buffer_epsilon[i];
	}
	result.distance = result.distance / bufferSize;
	result.angle = result.angle / bufferSize;

	return result;
}

ParallelController::orientationData ParallelController::getMedian()
{
	float copie_d[bufferSize];
	float copie_epsilon[bufferSize];
	orientationData result;

	for (int i = 0; i < bufferSize; i++)
	{
		copie_d[i] = buffer_D[i];
		copie_epsilon[i] = buffer_epsilon[i];
	}
	std::sort(copie_d, copie_d + bufferSize);
	std::sort(copie_epsilon, copie_epsilon + bufferSize);

	result.distance = copie_d[(bufferSize / 2)];
	result.angle = copie_epsilon[(bufferSize / 2)];

	return result;
}


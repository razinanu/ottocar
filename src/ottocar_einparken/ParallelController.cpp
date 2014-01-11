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
{ TARGET_DISTANCE, TARGET_DISTANCE, TARGET_DISTANCE, TARGET_DISTANCE, TARGET_DISTANCE };
float buffer_epsilon[] =
{ 0, 0, 0, 0, 0 };
int bufferSize = 1;
int bufferPointer = 0;
int obstacleInFrontOfTheCar = true;

ParallelController::ParallelController()
{

}

ParallelController::~ParallelController()
{

}

//todo Hindernis am Ende der Strecke beruecksichtigen
//todo Bot steht an der Wand und findet keinen Sprung
void ParallelController::laserScanParallel(const sensor_msgs::LaserScan laser)
{
	int firstPointFirstCarton;
	int lastPoint;
	int pointCartonStart;
	int indexOfFirstMeasurement = 0;
	int firstPointLastCarton;
	triangleData triangle;

#if LASER_DATA == true
	for (int i = 0; i < (255); i++)
	{
		ROS_INFO(
				"i: %d: %2.4f | %2.4f", i, laser.ranges[511 - i], ((TARGET_DISTANCE / cos(i * laser.angle_increment)) * TOLERANCE));
	}
#endif

//	ROS_INFO("i: %d: %2.4f", 254, laser.ranges[511 - 254]);

//	//pruefen, ob Bot neben dem Karton steht
//	if (laser.ranges[laser.ranges.size() - 1] < TARGET_DISTANCE * TOLERANCE)
//	{
//		//ersten Sprung finden
//		lastPoint = findEdge(laser, 0);
//		if (lastPoint >= SEARCH_SPACE)
//		{
//			ROS_ERROR("[PLC]: Bot steht neben einem Karton; kein Sprung gefunden");
//			return;
//		}
//
//		//lokales Minimum finden
//		firstPointFirstCarton = findMinimum(laser, 0, lastPoint);
//		if (firstPointFirstCarton <= 0)
//		{
//			ROS_ERROR("[PLC]: Bot steht neben einem Karton, kein Minimum gefunden");
//			return;
//		}
//	}
//	else //Bot steht NICHT neben dem Karton
//	{


	//pruefen, ob vor dem Auto ein Hindernis steht (nicht elegant)
	if (laser.ranges[laser.ranges.size() - (255 + 1)] < 0.3)
	{
//#if INFO == true
		if (!obstacleInFrontOfTheCar)
		{
			ROS_WARN("[PLC]: Hindernis erkannt");
		}
//#endif
		obstacleInFrontOfTheCar = true;
	}
	else
	{
		obstacleInFrontOfTheCar = false;
	}


//ersten gueltigen Wert finden
	pointCartonStart = findCarton(laser, indexOfFirstMeasurement);
	if (pointCartonStart < 0)
	{
		ROS_ERROR("[PLC]: keinen ersten Karton gefunden");
		return;
	}

	//ersten Sprung finden
	lastPoint = findEdge(laser, pointCartonStart);
	if (lastPoint < 0)
	{
		ROS_ERROR("[PLC]: beim 1. Karton keinen Sprung gefunden");
		return;
	}

	//lokales Minimum finden
	firstPointFirstCarton = findMinimum(laser, pointCartonStart, lastPoint);
	if (firstPointFirstCarton <= 0)
	{
		ROS_ERROR("[PLC]: kein Minimum beim 1. Karton gefunden");
		return;
	}
//	}

//	triangle = calculateTriangle(laser, point_B, point_C);
	triangle = calculateBetterTriangle(laser, firstPointFirstCarton, lastPoint);

	//Bedingung a*k <= f pruefen
	int cartonCounter = 1;
	while ((triangle.side_a * CONFIDENCE < triangle.side_f)
			|| (triangle.side_a < 0.1) || (triangle.side_b <= triangle.side_c)) //todo magischer Wert
	{
		cartonCounter++;

		//naechsten gueltigen Wert finden
		pointCartonStart = findCarton(laser, lastPoint + 1);
		if (pointCartonStart < 0)
		{
			ROS_ERROR("[PLC]: keinen %d. Karton gefunden", cartonCounter);
			return; //todo sollte hier nicht abbrechen
		}

		//naechsten Sprung finden
		lastPoint = findEdge(laser, pointCartonStart);

		//minimum des naechsten Sprunges finden
		firstPointLastCarton = findMinimum(laser, pointCartonStart, lastPoint);
		if (firstPointLastCarton < 0)
		{
			ROS_ERROR(
					"[PLC]: kein Minimum beim %d. Karton gefunden", cartonCounter);
			return; //todo sollte hier nicht abbrechen
		}

//		triangle = calculateTriangle(laser, point_B, point_C);
		triangle = calculateBetterTriangle2(laser, firstPointFirstCarton,
				lastPoint, firstPointLastCarton, triangle.side_c);

#if INFO == true
		ROS_INFO("[PLC]: point_b: %d", firstPointFirstCarton);
		ROS_INFO("[PLC]: point_c: %d", lastPoint);
#endif
	}

	writeToBuffer(triangle.side_d, triangle.epsilon);

#if INFO == true
	ROS_INFO("[PLC]: point_b: %d", firstPointFirstCarton);
	ROS_INFO("[PLC]: point_c: %d", lastPoint);
	ROS_INFO("[PLC]: side_d: %2.8f", triangle.side_d);
	ROS_WARN("[PLC]: side_d Median: %2.8f", getMedian().distance);
	ROS_WARN(
			"[PLC]: epsilon Median: %2.8f (%2.2f Grad)", getMedian().angle, getMedian().angle * 180 / M_PI);
	ROS_INFO("[PLC]: -------------------------------------- ");
#endif
//	ROS_INFO("[PLC]: side_d Median: %2.8f", getMedian().distance);
}

ParallelController::triangleData ParallelController::calculateTriangle(
		const sensor_msgs::LaserScan &laser, int firstPoint, int lastPoint)
{
	ROS_WARN("[PLC]: calculateTriangle sollte nicht benutzt werden");

	if (firstPoint >= lastPoint)
	{
		ROS_ERROR("[PLC]: calculateTriangle falsch aufgerufen");
	}

	triangleData result;
	result.side_c = laser.ranges[laser.ranges.size() - (firstPoint + 1)];
	result.side_b = laser.ranges[laser.ranges.size() - (lastPoint + 1)];

	/* a = sqrt(b² + c² -2bc * cos alpha) */
	result.side_a = sqrt(
			(pow(result.side_b, 2) + pow(result.side_c, 2))
					- (2 * result.side_b * result.side_c
							* cos(
									(lastPoint - firstPoint)
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
	result.epsilon = (firstPoint * laser.angle_increment) - result.mue;

#if INFO == true
	ROS_INFO("[PLC]: side_c: %2.8f", result.side_c);
	ROS_INFO("[PLC]: side_b: %2.8f", result.side_b);
	ROS_INFO("[PLC]: side_a: %2.8f", result.side_a);
	ROS_INFO(
			"[PLC]: alpha: %2.8f", ((lastPoint - firstPoint) * laser.angle_increment));
	ROS_INFO("[PLC]: beta: %2.8f", result.beta);
	ROS_INFO("[PLC]: lambda: %2.8f", result.lambda);
	ROS_INFO("[PLC]: side_f: %2.8f", result.side_f);
	ROS_INFO("[PLC]: side_d: %2.8f", result.side_d);
	ROS_INFO("[PLC]: mue: %2.8f", result.mue);
	ROS_INFO("[PLC]: epsilon: %2.8f", result.epsilon);
#endif

	return result;
}

ParallelController::triangleData ParallelController::calculateBetterTriangle(
		const sensor_msgs::LaserScan &laser, int firstPoint, int lastPoint)
{
	triangleData result;
	triangleSide help;
	float gamma;

	if (lastPoint <= firstPoint)
	{
		ROS_ERROR("[PLC]: calculateBetterTriangle falsch aufgerufen");
		return result;
	}

//	help = calculateRegressionLine(firstPoint, lastPoint, firstPoint, lastPoint, laser);
//	result.side_b = help.lastPoint;
//	result.side_c = help.firstPoint;

	result.side_b = laser.ranges[laser.ranges.size() - (lastPoint + 1)];
	result.side_c = laser.ranges[laser.ranges.size() - (firstPoint + 1)];

	/* a = sqrt(b² + c² -2bc * cos alpha) */
	result.side_a = sqrt(
			(pow(result.side_b, 2) + pow(result.side_c, 2))
					- (2 * result.side_b * result.side_c
							* cos(
									(lastPoint - firstPoint)
											* laser.angle_increment)));

	/* beta = acos((b²-a²-c²)/-2ac) */
	result.beta = acos(
			(pow(result.side_b, 2) - pow(result.side_a, 2)
					- pow(result.side_c, 2))
					/ ((-2) * result.side_a * result.side_c));

	//gamma
	gamma = M_PI - result.beta
			- ((lastPoint - firstPoint) * laser.angle_increment);

	//d
	result.side_d = sin(gamma) * result.side_b;

	//f
	result.side_f = (cos(gamma) * result.side_b) - result.side_a;

	//epsilon
	result.epsilon = acos(result.side_d / result.side_b)
			- lastPoint * laser.angle_increment;

#if INFO == true
	ROS_INFO("[PLC]: side_c: %2.8f", result.side_c);
	ROS_INFO("[PLC]: side_b: %2.8f", result.side_b);
	ROS_INFO("[PLC]: side_a: %2.8f", result.side_a);
	ROS_INFO(
			"[PLC]: alpha: %2.8f", ((lastPoint - firstPoint) * laser.angle_increment));
	ROS_INFO("[PLC]: beta: %2.8f", result.beta);
	ROS_INFO("[PLC]: gamma: %2.8f", gamma);
	ROS_INFO("[PLC]: side_f: %2.8f", result.side_f);
	ROS_INFO("[PLC]: side_d: %2.8f", result.side_d);
	ROS_INFO("[PLC]: epsilon: %2.8f", result.epsilon);
#endif

	return result;
}

ParallelController::triangleData ParallelController::calculateBetterTriangle2(
		const sensor_msgs::LaserScan &laser, int firstPoint, int lastPoint,
		int firstPointLastCarton, float side_C)
{
	triangleData result;
	float gamma;

	if (lastPoint <= firstPoint)
	{
		ROS_ERROR("[PLC]: calculateBetterTriangle2 falsch aufgerufen");
		return result;
	}

//	result.side_b = calculateRegressionLine(firstPointLastCarton, lastPoint, lastPoint, lastPoint, laser).lastPoint;
	result.side_c = side_C;

	result.side_b = laser.ranges[laser.ranges.size() - (lastPoint + 1)];

	/* a = sqrt(b² + c² -2bc * cos alpha) */
	result.side_a = sqrt(
			(pow(result.side_b, 2) + pow(result.side_c, 2))
					- (2 * result.side_b * result.side_c
							* cos(
									(lastPoint - firstPoint)
											* laser.angle_increment)));

	/* beta = acos((b²-a²-c²)/-2ac) */
	result.beta = acos(
			(pow(result.side_b, 2) - pow(result.side_a, 2)
					- pow(result.side_c, 2))
					/ ((-2) * result.side_a * result.side_c));

	//gamma
	gamma = M_PI - result.beta
			- ((lastPoint - firstPoint) * laser.angle_increment);

	//d
	result.side_d = sin(gamma) * result.side_b;

	//f
	result.side_f = (cos(gamma) * result.side_b) - result.side_a;

	//epsilon
	result.epsilon = acos(result.side_d / result.side_b)
			- lastPoint * laser.angle_increment;

#if INFO == true
	ROS_INFO("[PLC]: side_c: %2.8f", result.side_c);
	ROS_INFO("[PLC]: side_b: %2.8f", result.side_b);
	ROS_INFO("[PLC]: side_a: %2.8f", result.side_a);
	ROS_INFO(
			"[PLC]: alpha: %2.8f", ((lastPoint - firstPoint) * laser.angle_increment));
	ROS_INFO("[PLC]: beta: %2.8f", result.beta);
	ROS_INFO("[PLC]: gamma: %2.8f", gamma);
	ROS_INFO("[PLC]: side_f: %2.8f", result.side_f);
	ROS_INFO("[PLC]: side_d: %2.8f", result.side_d);
	ROS_INFO("[PLC]: epsilon: %2.8f", result.epsilon);
#endif

	return result;
}

ParallelController::triangleSide ParallelController::calculateRegressionLine(
		int start, int end, int firstPoint, int lastPoint,
		const sensor_msgs::LaserScan &laser)
{
	//y = ax + b
	triangleSide result;
	float sumxiyi;
	float sumxi;
	float sumyi;
	float sumxi2;

	//n
	int n = end - start + 1;

	if (n < 2)
	{
		ROS_INFO("[PLC]: start: %d", start);
		ROS_INFO("[PLC]: end: %d", end);
		ROS_ERROR("[PLC]: calculateRegressionLine falsch aufgerufen");
		return result;
	}

	if (n == 2)
	{
		ROS_WARN("[PLC]: Regressionslinie aus nur 2 Punkten");
	}

	//sum (xi * yi)
	sumxiyi = 0;
	for (int i = start; i < end; i++)
	{
		sumxiyi += i * laser.ranges[laser.ranges.size() - (i + 1)];
	}

	//sum (xi), sum (yi)
	sumxi = 0;
	for (int i = start; i < end; i++)
	{
		sumxi += i;
	}

	sumyi = 0;
	for (int i = start; i < end; i++)
	{
		sumyi += laser.ranges[laser.ranges.size() - (i + 1)];
	}

	//sum (xi²)
	sumxi2 = 0;
	for (int i = start; i < end; i++)
	{
		sumxi2 += i * i;
	}

	//a
	float a = ((n * sumxiyi) - (sumxi * sumyi))
			/ ((n * sumxi2) - (sumxi * sumxi));

	//b
	float b = ((sumxi2 * sumyi) - (sumxi * sumxiyi))
			/ ((n * sumxi2) - (sumxi * sumxi));

	//result
	result.firstPoint = a * firstPoint + b;
	result.lastPoint = a * lastPoint + b;

#if INFO == true
	ROS_INFO("[PLC]: start: %d", start);
	ROS_INFO("[PLC]: end: %d", end);
	ROS_INFO("[PLC]: n: %d", n);
//	ROS_INFO("[PLC]: sumxiyi: %2.8f", sumxiyi);
//	ROS_INFO("[PLC]: sumxi: %2.8f", sumxi);
//	ROS_INFO("[PLC]: sumyi: %2.8f", sumyi);
//	ROS_INFO("[PLC]: sumxi2: %2.8f", sumxi2);
//	ROS_INFO("[PLC]: a: %2.8f", a);
//	ROS_INFO("[PLC]: b: %2.8f", b);
	ROS_INFO("[PLC]: result.firstPoint: %2.8f", result.firstPoint);
	ROS_INFO("[PLC]: result.lastPoint: %2.8f", result.lastPoint);
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
			&& (laser.ranges[laser.ranges.size() - (index + 1)] < MAX_RANGE)
			&& (index < SEARCH_SPACE))
	{
		index++;
	}

	if (index > SEARCH_SPACE)
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

	for (int i = index; i < SEARCH_SPACE; i++)
	{
		/*
		 * Karton gefunden, wenn:
		 * - Wert < perfekter Wert * Tolleranz &&
		 * - perfekter Wert < maximaler Wert &&
		 * - Wert > minimaler Wert
		 */
		if ((laser.ranges[laser.ranges.size() - (i + 1)]
				< (TARGET_DISTANCE / cos(i * laser.angle_increment)) * TOLERANCE)
				&& ((TARGET_DISTANCE / cos(i * laser.angle_increment))
						* TOLERANCE) < MAX_RANGE
				&& (laser.ranges[laser.ranges.size() - (i + 1)] > MIN_RANGE))
		{
			return i;
		}
	}

	return -1;
}

int ParallelController::findMinimum(const sensor_msgs::LaserScan &laser,
		int firstPoint, int lastPoint)
{
	float minimum;
	int indexMinimum;

	if (firstPoint < 0)
	{
		ROS_ERROR("[PLC]: findMinimum falsch aufgerufen; %d", firstPoint);
		return -1;
	}

	minimum = laser.ranges[laser.ranges.size() - (lastPoint + 1)];
	indexMinimum = lastPoint;

	for (int i = lastPoint - 1; i >= firstPoint; i--)
	{
		if ((laser.ranges[laser.ranges.size() - (i + 1)] < minimum)
				&& laser.ranges[laser.ranges.size() - (i + 1)] > MIN_RANGE)
		{
			minimum = laser.ranges[laser.ranges.size() - (i + 1)];
			indexMinimum = i;
		}
	}

	if (indexMinimum < 0)
	{
		ROS_ERROR("[PLC]: Fehler in findMinimum");
		return -1;
	}

	return indexMinimum; //todo ein magischer Wert, um nicht genau die Ecke zu treffen
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

void ParallelController::printBuffer()
{
	for (int i = 0; i < bufferSize; i++)
	{
		ROS_INFO("[PLC]: buffer_D[%d]: %2.8f", i, buffer_D[i]);
	}

	for (int i = 0; i < bufferSize; i++)
	{
		ROS_INFO("[PLC]: buffer_epsilon[%d]: %2.8f", i, buffer_epsilon[i]);
	}
}

bool ParallelController::driveEnable()
{
	return !obstacleInFrontOfTheCar;
}

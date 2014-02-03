/*
 * ParallelController.h
 *
 *  Created on: Dec 6, 2013
 *      Author: licht
 */


//INFO auf 'true' setzen, um Zwischenwerte der Berechnungen auszugeben
#ifndef INFO
#define INFO false
#endif

//LASER_DATA auf 'true' setzen, um Zwischenwerte der Berechnungen auszugeben
#ifndef LASER_DATA
#define LASER_DATA false
#endif

#ifndef PARALLELCONTROLLER_H_
#define PARALLELCONTROLLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "ConstPark.h"
#include <iostream>

class ParallelController
{
private:
	bool obstacleInFrontOfTheCar;

	void check1(const sensor_msgs::LaserScan laser);
	void check2();

public:

	ParallelController();
	virtual ~ParallelController();

	/**
	 * \brief  Werte des Laserscanners auswerten
	 *
	 * Diese Funktion berechnet bei jedem Aufruf den Abstand sowie die Ausrichtung des Autos zu den Kartons auf der rechten Seite
	 * des Fahrzeugs. Die Ergebnisse werden in der Klasse gespeichert und koennen ueber die entsprechenden Funktionen
	 * ausgelesen werden.
	 *
	 * \param		laser	Datenarray des Laserscanners
	 * \return    	void
	 *
	 */
	void laserScanParallel(const sensor_msgs::LaserScan laser);

	bool driveEnable();
};

#endif /* PARALLELCONTROLLER_H_ */

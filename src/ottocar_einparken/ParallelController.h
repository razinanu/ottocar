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

/**
 * \brief  Klasse zur Kollisionserkennung
 *
 *         Diese Klasse stellt eine Funktion zur Verfügung, die die Daten
 *         das Laserscanners dahingehend auswertet, ob sich ein Hindernis
 *         vor dem Auto befindet.
 *         Die ursprünglich angedachte Funktionalität, die Ausrichtung des
 *         Fahrzeuges bezüglich der Kartons zu bestimmen, erwies sich als
 *         nicht praktikabel.
 *
 */

class ParallelController
{
private:
	bool obstacleInFrontOfTheCar;

	///gibt die gesamten Messdaten des Laserscanners auf der Konsole aus
	void check1(const sensor_msgs::LaserScan laser);
	///gibt eine Rückmeldung auf der Konsole aus, wenn ein Hindernis erkannt wurde
	void check2();

public:

	ParallelController();
	virtual ~ParallelController();

	/**
	 * \brief  Werte des Laserscanners auswerten
	 *
	 * 		Diese Funktion wertet die Daten des Laserscanners dahingehend aus,
	 * 		ob sich ein Hindernis vor dem Auto befindet.
	 *
	 * \param		laser	Datenarray des Laserscanners
	 *
	 */
	void laserScanParallel(const sensor_msgs::LaserScan laser);

	/**
	 * \brief  Fahrtfreigabe
	 *
	 *         Diese Funktion gibt den Wert true zurück, wenn sich kein Hindernis
	 *         vor dem Auto befindet.
	 */
	bool driveEnable();
};

#endif /* PARALLELCONTROLLER_H_ */

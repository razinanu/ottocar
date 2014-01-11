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
public:

	ParallelController();
	virtual ~ParallelController();

	struct triangleData
	{
		float side_a;
		float side_b;
		float side_c;
		float side_d;
		float side_f;

		float beta;
		float lambda;
		float mue;
		float epsilon;
	};

	struct orientationData
	{
		float distance;
		float angle;
	};

	struct triangleSide
	{
		float firstPoint;
		float lastPoint;
	};

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

	/**
	 * \brief  Rueckgabe der Ausrichtung - Mittelwert
	 *
	 * Diese Funktion gibt den (fliessenden) Mittelwert der relevanten Daten fuer die Ausrichtung zurueck. Dieses ist zum einen
	 * der seitliche Abstand zu den Kartons (side_d), sowie der Winkel zu den Kartons (epsilon). Diese Werte sind berechnet und
	 * nicht gemessen. Schwankungen von ca. +-1cm sowie +-5° bei unveraenderter Position sind normal.
	 *
	 * \return    	orientationData		seitlicher Abstand zu den Kartons in Metern sowie der Winkel zu den Kartons in Grad
	 *
	 */
	orientationData getAverage();

	/**
	 * \brief  Rueckgabe der Ausrichtung - Median
	 *
	 * Diese Funktion gibt den (fliessenden) Median der relevanten Daten fuer die Ausrichtung zurueck. Dieses ist zum einen
	 * der seitliche Abstand zu den Kartons (side_d), sowie der Winkel zu den Kartons (epsilon). Diese Werte sind berechnet und
	 * nicht gemessen. Schwankungen von ca. +-1cm sowie +-5° bei unveraenderter Position sind normal.
	 *
	 * \return    	orientationData		seitlicher Abstand zu den Kartons in Metern sowie der Winkel zu den Kartons in Grad
	 *
	 */
	orientationData getMedian();

	///gibt false zurueck, wenn vor dem Auto ein Hindernis steht
	bool driveEnable();

private:

	/**
	 * \brief  Berechnung der Dreiecke
	 *
	 * Diese Funktion berechnet alle Werte des Dreiecks, welches zwischen dem Laserscanner und
	 * den Punkten B und C aufgespannt wird. Die einzelnen Bezeichnungen sind der Skizze zu
	 * entnehmen.
	 *
	 * \param		laser	Datenarray des Laserscanners
	 * \param		point_B
	 * \param		point_C
	 * \return    	gibt saemtliche Laengen und Winkel der beiden Dreiecke zurueck
	 *
	 */
	triangleData calculateTriangle(const sensor_msgs::LaserScan &laser, int point_B, int point_C);

	triangleData calculateBetterTriangle(const sensor_msgs::LaserScan &laser, int firstPoint, int lastPoint);

	triangleData calculateBetterTriangle2(const sensor_msgs::LaserScan &laser, int firstPoint, int lastPoint, int firstPointLastCarton, float side_C);

	/**
	 * \brief  Suche des Endes eines Kartons
	 *
	 * Diese Funktion sucht in den uebergebenen Laserscandaten nach dem Ende eines gefundenen Kartons. Die Suche
	 * startet bei dem uebergebenen Index und bricht beim Ende des Kartons ab. Zurueckgegeben wird
	 * der Index des Laserstrahls, bei dem das Ende gefunden wurde. Ist die Suche erfolglos, gibt
	 * die Funktion den Wert '-1' zurueck.
	 *
	 * \param		laser	Datenarray des Laserscanners
	 * \param		index	Index, bei dem die Suche beginnen soll; ein Wert zwischen 256 und 511
	 * \return    	gibt den Index zurueck, bei dem Das Ende des Kartons gefunden wurde oder -1
	 *
	 */
	int findEdge(const sensor_msgs::LaserScan &laser, int index);

	/**
	 * \brief  Suchen eines Kartons
	 *
	 * Diese Funktion sucht in den uebergebenen Laserscandaten nach einem Karton. Die Suche
	 * startet bei dem uebergebenen Index und bricht am Anfang eines gefundenen Kartons ab. Zurueckgegeben wird
	 * der Index des Laserstrahls, bei dem der Karton gefunden wurde. Ist die Suche erfolglos, gibt
	 * die Funktion den Wert '-1' zurueck.
	 *
	 * \param		laser	Datenarray des Laserscanners
	 * \param		index	Index, bei dem die Suche beginnen soll; ein Wert zwischen 256 und 511
	 * \return    	gibt den Index zurueck, bei dem ein Karton gefunden wurde oder -1
	 *
	 */
	int findCarton(const sensor_msgs::LaserScan &laser, int index);

	/**
	 * \brief  Suchen eines lokalen Minimums
	 *
	 * Diese Funktion sucht in den uebergebenen Laserscandaten nach einem lokalen Minimum. Die Suche
	 * startet bei dem uebergebenen Index und bricht bei einem lokalen Minimum ab. Zurueckgegeben wird
	 * der Index des Laserstrahls, bei dem das Minimum gefunden wurde. Ist die Suche erfolglos, gibt
	 * die Funktion den Wert '-1' zurueck.
	 *
	 * \param		laser	Datenarray des Laserscanners
	 * \param		index	Index, bei dem die Suche beginnen soll; ein Wert zwischen 256 und 511
	 * \return    	gibt den Index zurueck, bei dem ein lokales Minimum gefunden wurde oder -1
	 *
	 */
	int findMinimum(const sensor_msgs::LaserScan &laser, int firstPoint, int lastPoint);

	/**
	 * \brief Berechnung einer Regressionsgeraden
	 *
	 * Diese Funktion berechnet eine Regressionsgerade fuer die uebergebenen Laserdaten zwischen
	 * dem Start- und Endpunkt. Zurueckgegeben wird der Y-Wert des Punktes 'point', der auf dieser
	 * Geraden liegt.
	 *
	 * \param		start	erster Punkt, der fuer die Gerade verwendet wird
	 * \param		end		letzter Punkt, der fuer die Gerade verwendet wird; end > start
	 * \param		point	der Punkt auf der Geraden, dessen Y-Wert zurueckgegeben werden soll; start <= point <= end
	 * \param		laser	Datenarray des Laserscanners
	 */
	float calculateRegressionLine(int start, int end, int point, const sensor_msgs::LaserScan &laser);

	triangleSide calculateRegressionLine(int start, int end, int pointB, int pointC, const sensor_msgs::LaserScan &laser);

	///schreibt die uebergebenen Werte in den Ringpuffer
	void writeToBuffer(float value_D, float value_epsilon);

	///gibt den Inhalt des Buffers auf der Konsole aus
	void printBuffer();

	///liefert true zurueck, wenn der Bot neben einem Karton steht
	bool botBesideCarton();
};

#endif /* PARALLELCONTROLLER_H_ */

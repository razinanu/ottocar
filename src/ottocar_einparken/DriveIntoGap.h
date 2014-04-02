/*
 * DriveIntoGap.h
 *
 *  Created on: 06.12.2013
 *      Author: jsabsch
 */

#ifndef DRIVEINTOGAP_H_
#define DRIVEINTOGAP_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

 /**
  * \brief  Diese Klasse ist für den gesamten Einparkvorgang zuständig
  *
  *         Diese Klasse bildet den gasamten Vorgang des Einparkens ab. Dieser
  *         Vorgang läuft streng sequentiell ab. Die Ausführung der einzelnen
  *         Schritte wird durch Sensordaten ausgelöst und ist größtenteils nicht
  *         zeitabhängig.
  *
  */

class DriveIntoGap
{
public:
	// GLOBAL
	bool blinkDone;

	///Datentyp, um Fahrdaten sowie 3 LEDs zu übergeben
	struct twoInts
	{
		int angle;
		int speed;
		int led1;
		int led2;
		int led3;
	};

	///Datentyp, um Fahrdaten sowie alle LEDs zu übergeben
	struct driveData
	{
		std_msgs::Int8 speed;
		std_msgs::Int8 angle;
		std_msgs::UInt8 led0;
		std_msgs::UInt8 led1;
		std_msgs::UInt8 led2;
		std_msgs::UInt8 led3;
		std_msgs::UInt8 led4;
		std_msgs::UInt8 led5;
		std_msgs::UInt8 led6;
		std_msgs::UInt8 led7;
		std_msgs::UInt8 led8;
	};

	DriveIntoGap();
	virtual ~DriveIntoGap();

	 /**
	  * \brief  Diese Funktion ist für den gesamten Einparkvorgang zuständig
	  *
	  *         Diese Funktion steuert den gesamten Einparkvorgang und ist streng
	  *         sequentiell aufgebaut. Die Übergänge zwischen den einzelnen Phasen
	  *         des Einparkens werden durch jeweils spezifizierte Sensoreingaben
	  *         ausgelöst. Zurückgegeben werden die Geschwindigkeit und der Lenk-
	  *         winkel, die in der Parking.cpp gepublished werden.
	  *
	  * \param	laser   		Messwerte des Laserscanners
	  * \param	gapSize			Größe der Lücke, in die das Auto einparken soll (0,6/0,7/0,8)
	  * \param	distanceBack	Distanz des hinteren IR-Sensors in Metern
	  * \param	distanceSide	Distanz des seitlichen IR-Sensors in Metern
	  * \param	odometry		Werte der Odometrie
	  * \param	voltage			Spannung des Akkus in Volt
	  * \param	distanceToGap	Entfernung in Metern zu der Lücke, in die eingeparkt werden soll
	  *
	  *\return	Geschwindigkeit und Lenkwinkel, die gefahren werden sollen
	  *
	  */
	driveData  drive(sensor_msgs::LaserScan laser, float gapSize, float distanceBack, float distanceSide, int odometry, float voltage, float distanceToGap);


	int mode;
	bool ledChanged;

private:

	//float timeToDrive;
	ros::Time lastTime;
	int lastOdometry;
	int SPEED;

	///gibt die Geschwindigkeit in m/s in Abhängigkeit der Akkuspannung zurück --> recht unzuverlässig
	float calculateSpeed10(float voltage);

	///rechnet gefahrene Odometrieticks in Meter um
	float drivenM(int odometry);

	///0.5 Sekunden warten
	driveData wait1(int odometry);

	 /**
	  * \brief  Einparken Teil 1
	  *
	  *         Das Auto fährt in diesem Zustand mit vollem Lenkeinschlag den ersten
	  *         Bogen in die Parklücke (Räder nach rechts eingeschlagen). Die gefahrene
	  *         Strecke hängt dabei von der Lückengröße ab.
	  *
	  */
	driveData back1(float gapSize, int odometry);

	///0.5 Sekunden warten
	driveData wait2(int odometry);

	 /**
	  * \brief  Einparken Teil 2
	  *
	  *         Das Auto fährt in diesem Zustand mit vollem Lenkeinschlag den zweiten
	  *         Bogen in die Parklücke (Räder nach links eingeschlagen). Die gefahrene
	  *         Strecke hängt dabei von der Distanz zum vorderen Karton sowie der
	  *         Lückengröße ab. Da die Größe der Lücke sowie der Abstand zum vorderen
	  *         Karton bekannt sind, ergibt sich der Abstand zum hinteren Karton.
	  *
	  */
	driveData back2(const sensor_msgs::LaserScan laser, float distanceBack, int odometry, float gapSize);

	///0.3 Sekunden warten
	driveData wait3();

	///0.2 Sekunden warten und Lenkung einschlagen
	driveData waitTurn(int odometry);

	 /**
	  * \brief  Ausrichten
	  *
	  *         Das Auto fährt in diesem Zustand so weit wie möglich nach vorne. Je nach
	  *         Lückengröße variiert dabei der Lenkeinschlag. Der Lenkeinschlag ist dabei
	  *         jedoch fest vorgegeben und wird nicht durch die tatsächliche Ausrichtung
	  *         des Autos bestimmt.
	  *
	  */
	driveData forwards(const sensor_msgs::LaserScan laser, float gapSize, int odometry);

	///0.5 Sekunden warten
	driveData wait4(int odometry);

	 /**
	  * \brief  letzte Korrektur
	  *
	  *         Damit der Abstand zum vorderen Karton regelkomform ist, fährt das Auto
	  *         in diesem Zustand 5 cm zurück.
	  *
	  */
	driveData backLast(float distanceBack, int odometry);

	 /**
	  * \brief  Lücke Suchen
	  *
	  *         Das Programm befindet sich so lange in diesem Zustand, bis die
	  *         gewünschte Lücke gefunden wird. In diesem Zustand fährt das
	  *         Auto nur geradeaus.
	  *
	  */
	driveData waitForDistance(float distanceToGap, int odometry, float gapSize);

	 /**
	  * \brief  zur Lücke fahren
	  *
	  *         Wenn die Lücke gefunden wurde, befindet sich das Programm so
	  *         lange in diesem Zustand, bis sich der seitliche IR-Sensor in
	  *         der Mitte der Lücke befindet. In diesem Zustand fährt das
	  *         Auto nur geradeaus.
	  *
	  */
	driveData driveFirstHalf(int odometry, float dataIRside);

	 /**
	  * \brief  Ende der Lücke finden
	  *
	  *         Das Programm befindet sich so lange in diesem Zustand, bis das
	  *         Ende der Lücke durch den IR-Sensor detektiert wird. In diesem
	  *         Zustand fährt das Auto nur geradeaus.
	  *
	  */
	driveData driveSecondHalf(float dataIRside, int odometry);

	 /**
	  * \brief  x cm hinter Lücke halten
	  *
	  *         Nach der Detektion des Endes der gewünschten Lücke befindet
	  *         sich das Programm so lange in diesem Zustand, bis das Auto
	  *         x cm hinter der Lücke steht.
	  *
	  */
	driveData positioning(int odometry, float gapSize);

	bool blinkerOn;
	ros::Time lastTimeBlinkerChange;
	float distanceToDrive;
	ros::Time gapBegin;

	float foundGapSice;

	bool ledBlinkerRechts;
	bool ledBlinkerLinks;
	bool ledScheinwerferVorneRechts;
	bool ledScheinwerferVorneLinks;
	bool ledBremse;
};

#endif /* DRIVEINTOGAP_H_ */

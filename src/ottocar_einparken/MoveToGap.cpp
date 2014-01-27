/*
 * DriveToGap.cpp
 *
 *  Created on: 06.12.2013
 *      Author: licht
 */

#include "MoveToGap.h"

MoveToGap::MoveToGap()
{
	cartonSeen = false;
	distanceGot = false;
	gapBegin = ros::Time::now() - ros::Duration(2);
	mode = 0;
}

MoveToGap::~MoveToGap()
{
}

/// berechnet die Geschwindigkeit in m/s in Abhaengigkeit der Akkuspannung fuer die Geschwindigkeit -10
float MoveToGap::calculateSpeed10(float voltage)
{
	float result;
	result = 0.05083733 * voltage - 0.3547296;
	ROS_INFO("[MTG]: voltage: %2.4f | speed: %2.4f", voltage, result);
	return result;
}

MoveToGap::driveData MoveToGap::moveToGap(float dataIRside, float dataIRback, float distanceToGap, float voltage)
{
	driveData result;

	result.angle.data = STRAIGHTFORWARD;
	result.speed.data = -10;

//	ROS_INFO("distanceTOGap: %2.4f | IR: %2.4f", distanceToGap, dataIRside);

	switch(mode)
	{
	case 0:
	{
		//Auf eine Entfernung zur besten Lücke warten
		if (distanceToGap > 0 && distanceToGap < 1.0)	//todo ab wann den Wert akzeptieren?
		{
			ROS_INFO("[MTG]: distanceTOGap: %2.4f | IR: %2.4f", distanceToGap, dataIRside);
			distanceFound = ros::Time::now();
			timeToDrive = ((distanceToGap - 0.15) / calculateSpeed10(voltage));	//todo minus einen wert x
			mode = 1;
		}
		break;
	}
	case 1:
	{
		//darauf warten, dass das Auto in der besten Lücke steht
		if ((distanceFound + ros::Duration(timeToDrive)) < ros::Time::now())
		{
			ROS_INFO("[MTG]: In der Mitte neben der Luecke: %2.2f", dataIRside);
			mode = 2;
		}
		break;
	}
	case 2:
	{
		//auf das Ende der Lücke warten, bis der  IR-Sensor den Karton sieht
		if (dataIRside < 0.2)
		{
			ROS_INFO("[MTG]: Ende der Luecke gesehen: %2.2f", dataIRside);
			gapBegin = ros::Time::now();
			timeToDrive = (0.15 / calculateSpeed10(voltage));
			mode = 3;
		}
		break;
	}
	case 3:
	{
		//verzögert hinter der Lücke anhalten
		if ((gapBegin + ros::Duration(timeToDrive)) < ros::Time::now())
		{
			ROS_INFO("[MTG]: hinter der Luecke angehalten");
			result.speed.data = 0;
			lastTime = ros::Time::now();
			mode = 4;
		}
		break;
	}
//	case 4:
//	{
//		//warten
//		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 5;
//		}
//		result.speed.data = 0;
//		break;
//	}
//	case 5:
//	{
//		//rückwärts in die Parklücke einfahren 1. Teil
//		if ((lastTime + ros::Duration(1.6)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 6;
//		}
//		result.angle.data = RIGHT_MAX;
//		result.speed.data = 8;
//		break;
//	}
//	case 6:
//	{
//		//warten
//		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 7;
//		}
//		result.speed.data = 0;
//		break;
//	}
//	case 7:
//	{
//		//rückwärts in die Parklücke einfahren 2. Teil
//		if (dataIRback < 11)
//		{
//			lastTime = ros::Time::now();
//			mode = 8;
//		}
//		result.angle.data = LEFT_MAX;
//		result.speed.data = 7;
//		break;
//	}
//	case 8:
//	{
//		//warten
//		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 9;
//		}
//		result.speed.data = 0;
//		break;
//	}
//	case 9:
//	{
//		//kurz vorwärts fahren
//		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 10;
//		}
//		result.angle.data = RIGHT_MAX;
//		result.speed.data = - 8;
//		break;
//	}
//	case 10:
//	{
//		//warten
//		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 11;
//		}
//		result.angle.data = -21;
//		result.speed.data = 0;
//		break;
//	}

	default:
	{
		result.angle.data = STRAIGHTFORWARD;
		result.speed.data = 0;
		break;
	}
	}


	// einparken 70 cm
//	switch(mode)
//	{
//	case 0:
//	{
//		//Auf eine Entfernung zur besten Lücke warten
//		if (distanceToGap > 0 && distanceToGap < 3)
//		{
//			distanceFound = ros::Time::now();
//			timeToDrive = (distanceToGap / 0.4);	//todo minus einen wert x
//			mode = 1;
//		}
//		break;
//	}
//	case 1:
//	{
//		//darauf warten, dass das Auto in der besten Lücke steht
//		if ((distanceFound + ros::Duration(timeToDrive)) < ros::Time::now())
//		{
//			mode = 2;
//		}
//		break;
//	}
//	case 2:
//	{
//		//auf das Ende der Lücke warten, bis der  IR-Sensor den Karton sieht
//		if (dataIR < 12)
//		{
//			mode = 3;
//		}
//		gapBegin = ros::Time::now();
//		break;
//	}
//	case 3:
//	{
//		//verzögert hinter der Lücke anhalten
//		if (gapBegin < (ros::Time::now() - ros::Duration(0.4)))
//		{
//			result.speed.data = 0;
//			lastTime = ros::Time::now();
//			mode = 4;
//		}
//		break;
//	}
//	case 4:
//	{
//		//warten
//		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 5;
//		}
//		result.speed.data = 0;
//		break;
//	}
//	case 5:
//	{
//		//rückwärts in die Parklücke einfahren 1. Teil
//		if ((lastTime + ros::Duration(1.75)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 6;
//		}
//		result.angle.data = RIGHT_MAX;
//		result.speed.data = 8;
//		break;
//	}
//	case 6:
//	{
//		//warten
//		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 7;
//		}
//		result.speed.data = 0;
//		break;
//	}
//	case 7:
//	{
//		//rückwärts in die Parklücke einfahren 2. Teil
//		if ((lastTime + ros::Duration(1.25)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 8;
//		}
//		result.angle.data = LEFT_MAX;
//		result.speed.data = 8;
//		break;
//	}
//	case 8:
//	{
//		//warten
//		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 9;
//		}
//		result.speed.data = 0;
//		break;
//	}
//	case 9:
//	{
//		//kurz vorwärts fahren
//		if ((lastTime + ros::Duration(0.45)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 10;
//		}
//		result.angle.data = RIGHT_MAX;
//		result.speed.data = - 8;
//		break;
//	}
//	case 10:
//	{
//		//warten
//		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
//		{
//			lastTime = ros::Time::now();
//			mode = 11;
//		}
//		result.angle.data = -21;
//		result.speed.data = 0;
//		break;
//	}
//
//	default:
//	{
//		result.angle.data = -21;
//		result.speed.data = 0;
//		break;
//	}
//	}




	return result;
}

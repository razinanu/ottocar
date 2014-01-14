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

MoveToGap::driveData MoveToGap::moveToGap(float dataIRside, float dataIRback, float distanceToGap)
{
	driveData result;

	result.angle.data = -25;
	result.speed.data = -8;

	ROS_INFO("distanceTOGap: %2.4f | IR: %2.4f", distanceToGap, dataIRside);

	switch(mode)
	{
	case 0:
	{
		//Auf eine Entfernung zur besten Lücke warten
		if (distanceToGap > 0 && distanceToGap < 3)
		{
			distanceFound = ros::Time::now();
			timeToDrive = (distanceToGap / 0.4);	//todo minus einen wert x
			mode = 1;
		}
		break;
	}
	case 1:
	{
		//darauf warten, dass das Auto in der besten Lücke steht
		if ((distanceFound + ros::Duration(timeToDrive)) < ros::Time::now())
		{
			mode = 2;
		}
		break;
	}
	case 2:
	{
		//auf das Ende der Lücke warten, bis der  IR-Sensor den Karton sieht
		if (dataIRside < 12)
		{
			mode = 3;
		}
		gapBegin = ros::Time::now();
		break;
	}
	case 3:
	{
		//verzögert hinter der Lücke anhalten
		if (gapBegin < (ros::Time::now() - ros::Duration(0.45)))
		{
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
		result.angle.data = -21;
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

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <unistd.h>

#include <boost/filesystem.hpp>

#include "Robot/Robot.h"
#include "Debug/Debug.h"

using namespace std;
using namespace boost;

// Starting point for our robot program
// Testing comment
int main()
{
	try{
//		std::system("espeak \"I'm TAPAS\"");
		cout<<"Starting program" << endl;
		Robot robot("../settings.xml");
		cout << "Robot created" << endl;
		//Debug debug(&robot);
		//debug.testConstraints(filesystem::path("../MovementConstraints/Camera/database/train"),
		//						filesystem::path("../MovementConstraints/Camera/database/classify"));

//		double robotLat = GPS::decimal2Nmea(52.402357*100);
//		double robotLon = GPS::decimal2Nmea(16.952361*100);
//		printf("NMEA: %.8f \t\t %.8f\n", robotLat, robotLon);
//
//		robot.fakeGPSStart(robotLat, robotLon);
//
//		double lat = GPS::decimal2Nmea(52.402631*100);
//		double lon = GPS::decimal2Nmea(16.943022*100);
//
//		printf("1st NMEA: %.8f \t\t %.8f\n", lat, lon);
//
//		printf("DIFF: %.8f \t %.8f \t %.8f\n", 52.402357, 52.402631, GPS::decimal2radian(52.402631 - 52.402357));
//		printf("DIFF: %.8f \t %.8f \t %.8f\n", 16.952361, 16.943022, GPS::decimal2radian(16.952361 - 16.943022));
//
//
//		double x = robot.getPosX(lat)/1000;
//		double y = robot.getPosY(lon)/1000;
//
//		printf("1st xy: %.8f \t\t %.8f\n", x, y);
//
//		lat = GPS::decimal2Nmea(52.399394*100);
//		lon = GPS::decimal2Nmea(16.952256*100);
//
//		printf("2nd NMEA: %.8f \t\t %.8f\n", lat, lon);
//
//		double x2 = robot.getPosX(lat)/1000;
//		double y2 = robot.getPosY(lon)/1000;
//
//		printf("2nd xy: %.8f \t\t %.8f\n", x2, y2);
//
//		double dist = sqrt( pow(x - x2,2) + pow(y-y2,2));
//		printf("Distance check %f \n", dist);
//		scanf("%f", &lat);

		//debug.testEncoders();

//		robot.openImu("/dev/ttyACM0");
//		cout << "IMU has been opened!" << endl;

		char a;
		while(1)
		{
			cin >> a;
			if ( a == 'q')
				break;
		}

	}
	catch(char const* error){
		cout << error << endl;
	}
}


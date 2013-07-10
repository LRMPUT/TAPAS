#include <iostream>
#include <stdio.h>
#include <fstream>
#include <unistd.h>

#include "Robot/Robot.h"

using namespace std;

// Starting point for our robot program
// Testing comment
int main()
{
	Robot *ourRobot  = new Robot();


	// Streams to save data
	ofstream encoderStream, imuStream, gpsStream;
	encoderStream.open("encoder.data");
	imuStream.open("imu.data");
	gpsStream.open("gps.data");

	// Opening to capture data
	// Hardcoded devices -> ADJUST IT !
	ourRobot->openGps("/dev/ttyS0");
	ourRobot->openImu("/dev/ttyS1");
	ourRobot->openRobotsDrive("/dev/ttyS2");

	// Reading data
	if ( ourRobot->isImuOpen() && ourRobot->isGpsOpen() )
	{
		cv::Mat encoderData, gpsData, imuData;
		while(1)
		{
			encoderData = ourRobot->getEncoderData();
			gpsData = ourRobot->getGpsData();
			imuData = ourRobot->getImuData();

			// enc data 2x1 - użyj operatora .at<int>()
			//encoderStream << encoderData[0]<< " "<<encoderData[1] << endl;

			// gps data 2x1
			//gpsStream << gpsData[0] << " " << gpsData[1] << endl;

			// I changed dimensions to be row-aligned -> each row represents data from 1 sensor
			// imu data 4x3
			for (int i = 0; i < 4; i++)
				for (int j=0; j<3; j++)
					//imuStream << imuData[i][j] << " ";
			imuStream << endl;

			// Waiting 50 ms to have some rational data stream catching -> max 20 Hz
			usleep(50 * 1000);

		}
	}

	// Closing streams
	encoderStream.close();
	gpsStream.close();
	imuStream.close();

	delete ourRobot;
}


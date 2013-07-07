#include <iostream>
#include <stdio.h>
#include <fstream>
#include <unistd.h>

#include "Robot/Robot.h"

// Starting point for our robot program
// Testing comment
int main()
{
	Robot *ourRobot  = new Robot();


	// Streams to save data
	ofstream encoderStream, imuStream, gpsStream;
	encoderStream.open("encoder.data");
	imuStream.open("imu.data");
	gpsStream.open("gps.data");'

	// Opening to capture data
	// Hardcoded devices -> ADJUST IT !
	Robot->openGps("/dev/ttyS0");
	Robot->openImu("/dev/ttyS1");
	Robot->openRobotsDrive("/dev/ttyS2");

	// Reading data
	if ( Robot->isImuOpen() && Robot->isGpsOpen() )
	{
		cv::Mat encoderData, gpsData, imuData;
		while(1)
		{
			encoderData = Robot->getEncoderData();
			gpsData = Robot->getGpsData();
			imuData = Robot->getImuData();

			// enc data 2x1
			encoderStream << encoderData[0]<< " "<<encoderData[1] << endl;

			// gps data 2x1
			gpsStream << gpsData[0] << " " << gpsData[1] << endl;

			// I changed dimensions to be row-aligned -> each row represents data from 1 sensor
			// imu data 4x3
			for (int i = 0; i < 4; i++)
				for (int j=0; j<3; j++)
					imuStream << imuData[i][j] << " ";
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

/*
 * robotDrivers.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: sebastian
 */

#include "robotDrivers.h"
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace cv;
using namespace trobot;



Drivers::Drivers() {


}

Drivers::Drivers(unsigned int baud, const std::string device) {

	serialPort = new SerialPort(baud, device);

}

Drivers::~Drivers() {

}

void Drivers::exitSafeStart(){

	bool commandCorrect = false;
	static const int countDriveLimit = 50;
	static const int errorCountLimit = 10;
	/// no response timeout counter
	int driveCount = 0;
	/// unknown command counter
	int errorCount1 = 0;
	/// wrong command counter
	int errorCount2 = 0;

	/// send save start command
	serialPort->write("GO\r");

	while(!commandCorrect && driveCount < 50){

		/// send command again if failure occured
		if (errorCount1 > 0 || errorCount2 > 0)
			serialPort->write("GO\r");

		/// read serial data
		driverResp = serialPort->getDataRead();

		if (searchBufferR(driverResp, "?") != -1  )
			errorCount1++;
		else if (searchBufferR(driverResp, "!") != -1  )
			errorCount2++;
		else if (searchBufferR(driverResp, ".") != -1  )
				commandCorrect = true;

		usleep(1000);
		driveCount++;

		if (errorCount1 > errorCountLimit ||
			errorCount2 > errorCountLimit)
			break;
	}

	if (driveCount == countDriveLimit)
		throw "No driver response";
	else if (errorCount1 == errorCountLimit)
		throw "Unknown command sent";
	else if (errorCount2 == errorCountLimit)
		throw "Driver error. Command known.";

}

void Drivers::setMotorSpeed(float speed){

	bool commandCorrect = false;
	static const int countDriveLimit = 50;
	static const int errorCountLimit = 10;
	/// no response timeout counter
	int driveCount = 0;
	/// unknown command counter
	int errorCount1 = 0;
	/// wrong command counter
	int errorCount2 = 0;

	if (speed > 0)
		serialPort->write('F' + boost::lexical_cast<string>(speed) + '\r');
	else
		serialPort->write('R' + boost::lexical_cast<string>(-speed) + '\r');

	while(!commandCorrect && driveCount < 50){

				/// send command again if failure occured
				if (errorCount1 > 0 || errorCount2 > 0){

					if (speed > 0)
						serialPort->write('F' + boost::lexical_cast<string>(speed) + '\r');
					else
						serialPort->write('R' + boost::lexical_cast<string>(-speed) + '\r');
				}

				/// read serial data
				driverResp = serialPort->getDataRead();

				if (searchBufferR(driverResp, "?") != -1  )
					errorCount1++;
				else if (searchBufferR(driverResp, "!") != -1  )
					errorCount2++;
				else if (searchBufferR(driverResp, ".") != -1  )
					commandCorrect = true;

				usleep(1000);
				driveCount++;

				if (errorCount1 > errorCountLimit ||
					errorCount2 > errorCountLimit)
					break;
	}

	if (driveCount == countDriveLimit)
		throw "No driver response";
	else if (errorCount1 == errorCountLimit)
		throw "Unknown command sent";
	else if (errorCount2 == errorCountLimit)
		throw "Driver error. Commands known.";

}

void Drivers::stopMotor(){

	bool commandCorrect = false;
	static const int countDriveLimit = 50;
	static const int errorCountLimit = 10;
	/// no response timeout counter
	int driveCount = 0;
	/// unknown command counter
	int errorCount1 = 0;
	/// wrong command counter
	int errorCount2 = 0;

	serialPort->write("X\r");

		while(!commandCorrect && driveCount < 50){

			/// send command again if failure occured
			if (errorCount1 > 0 || errorCount2 > 0)
				serialPort->write("X\r");

			/// read serial data
			driverResp = serialPort->getDataRead();

			if (searchBufferR(driverResp, "?") != -1  )
				errorCount1++;
			else if (searchBufferR(driverResp, "!") != -1  )
				commandCorrect = true;

			usleep(1000);
			driveCount++;

			if (errorCount1 > errorCountLimit ||
				errorCount2 > errorCountLimit)
				break;
	}

		if (driveCount == countDriveLimit)
			throw "No driver response";
		else if (errorCount1 == errorCountLimit)
			throw "Unknown command sent";
		else if (errorCount2 == errorCountLimit)
			throw "Driver error. Commands known.";
}


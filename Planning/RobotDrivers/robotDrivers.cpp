/*
	Copyright (c) 2014,	TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl),
	-Sebastian Bromberek.
	Poznan University of Technology
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice,
	this list of conditions and the following disclaimer in the documentation
	and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
	THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
	AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

	//cout << "Drivers::exitSafeStart()" << endl;
	while(!commandCorrect && driveCount < 50){

		/// send command again if failure occured
		if (errorCount1 > 0 || errorCount2 > 0)
			serialPort->write("GO\r");

		/// read serial data
		driverResp = serialPort->getDataRead();

		/*cout << "driverResp: ";
		for(int i = 0; i < driverResp.size(); i++){
			cout << driverResp[i];
		}
		cout << endl;*/

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

	if (driveCount == countDriveLimit){
		cout << "Drivers::exitSafeStart(): No driver response" << endl;
		throw "Drivers::exitSafeStart(): No driver response";
	}
	else if (errorCount1 == errorCountLimit){
		cout << "Drivers::exitSafeStart(): Unknown command sent" << endl;
		throw "Drivers::exitSafeStart(): Unknown command sent";
	}
	else if (errorCount2 == errorCountLimit){
		cout << "Drivers::exitSafeStart(): Driver error. Command known." << endl;
		throw "Drivers::exitSafeStart(): Driver error. Command known.";
	}

}

void Drivers::setMotorSpeed(int speed){

	bool commandCorrect = false;
	static const int countDriveLimit = 50;
	static const int errorCountLimit = 10;
	/// no response timeout counter
	int driveCount = 0;
	/// unknown command counter
	int errorCount1 = 0;
	/// wrong command counter
	int errorCount2 = 0;

	//cout << "speed = " << speed << endl;
	if (speed >= 0){
		//cout << 'F' + boost::lexical_cast<string>(speed) + '\r' << endl;
		serialPort->write('F' + boost::lexical_cast<string>(speed) + '\r');
	}
	else{
		//cout << 'R' + boost::lexical_cast<string>(-speed) + '\r' << endl;
		serialPort->write('R' + boost::lexical_cast<string>(-speed) + '\r');
	}

	//cout << "Drivers::setMotorSpeed()" << endl;
	while(!commandCorrect && driveCount < 50){

				/// send command again if failure occured
				if (errorCount1 > 0 || errorCount2 > 0){

					if (speed >= 0)
						serialPort->write('F' + boost::lexical_cast<string>(speed) + '\r');
					else
						serialPort->write('R' + boost::lexical_cast<string>(-speed) + '\r');
				}

				/// read serial data
				driverResp = serialPort->getDataRead();

				/*cout << "driverResp: ";
				for(int i = 0; i < driverResp.size(); i++){
					cout << driverResp[i];
				}
				cout << endl;*/

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

	if (driveCount == countDriveLimit){
		cout << "Drivers::setMotorSpeed: No driver response" << endl;
		throw "Drivers::setMotorSpeed: No driver response";
	}
	else if (errorCount1 == errorCountLimit){
		cout << "Drivers::setMotorSpeed: Unknown command sent" << endl;
		throw "Drivers::setMotorSpeed: Unknown command sent";
	}
	else if (errorCount2 == errorCountLimit){
		cout << "Drivers::setMotorSpeed: Driver error. Command known." << endl;
		throw "Drivers::setMotorSpeed: Driver error. Command known.";
	}

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


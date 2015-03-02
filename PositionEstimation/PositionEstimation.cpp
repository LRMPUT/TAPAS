/*
	Copyright (c) 2014,	TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
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

#include <opencv2/opencv.hpp>
#include <string>
#include "PositionEstimation.h"
#include <thread>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

using namespace cv;
using namespace std;

PositionEstimation::PositionEstimation(Robot* irobot, TiXmlElement* settings) :
		robot(irobot), imu(irobot), gps(irobot), encoders(irobot) {
	std::cout << "PositionEstimation::PositionEstimation" << std::endl;

	readSettings(settings);

	// Open Log file
	logStream.open("positionEstimation.log");
	logGpxStream.open("positionEstimation.gpx");

	/* KALMAN:
	 * - we track 2 values -> global position
	 *
	 * - to predict we can use values from encoders
	 * - to correct we can use information from the GPS
	 *
	 */
	kalmanSetup();

	lastUpdateTimestamp = std::chrono::high_resolution_clock::now();
	lastEncoderTimestamp = std::chrono::high_resolution_clock::now();
	lastGpsTimestamp = std::chrono::high_resolution_clock::now();
	lastImuTimestamp = std::chrono::high_resolution_clock::now();

	estimationThread = std::thread(&PositionEstimation::run, this);

	std::cout << "End PositionEstimation::PositionEstimation" << std::endl;
}

PositionEstimation::~PositionEstimation() {
	cout << "~PositionEstimation()" << endl;
	stopThread();
	closeGps();
	closeImu();
	delete EKF;
	logStream.close();
	logGpxStream << "</trkseg></trk></gpx>" << std::endl;
	logGpxStream.close();
	cout << "End ~PositionEstimation()" << endl;
}

void PositionEstimation::readSettings(TiXmlElement* settings) {
	TiXmlElement* pPositionEstimation = settings->FirstChildElement(
			"PositionEstimation");
	if (pPositionEstimation->QueryIntAttribute("runThread",
			&parameters.runThread) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for position estimation Thread";
	}
	if (pPositionEstimation->QueryDoubleAttribute("processingFrequency",
			&parameters.processingFrequency) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for positionEstimation processing frequency";
	}
	if (pPositionEstimation->QueryIntAttribute("debug", &parameters.debug)
			!= TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for position estimation debug";
	}
	if (pPositionEstimation->QueryIntAttribute("encoderTicksPerRev",
			&parameters.encoderTicksPerRev) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for position estimation encoderTicksPerRev";
	}
	if (pPositionEstimation->QueryDoubleAttribute("wheelDiameter",
			&parameters.wheelDiameter) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for position estimation wheel diameter";
	}
	if (pPositionEstimation->QueryDoubleAttribute("wheelBase",
			&parameters.wheelBase) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for positionEstimation wheel base";
	}
	if (pPositionEstimation->QueryDoubleAttribute("predictionVariance",
			&parameters.predictionVariance) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for positionEstimation predictionVariance";
	}
	if (pPositionEstimation->QueryDoubleAttribute("gpsVariance",
			&parameters.gpsVariance) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for positionEstimation gpsVariance";
	}
	if (pPositionEstimation->QueryDoubleAttribute("imuVariance",
			&parameters.imuVariance) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for positionEstimation imuVariance";
	}
	if (pPositionEstimation->QueryDoubleAttribute("encoderVariance",
			&parameters.encoderVariance) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for positionEstimation encoderVariance";
	}

	TiXmlElement* pGlobalPlanner = settings->FirstChildElement("GlobalPlanner");
	if (pGlobalPlanner->QueryDoubleAttribute("goalLatitude",
			&parameters.goalLatitude) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner goalLatitude";
	}

	if (pGlobalPlanner->QueryDoubleAttribute("goalLongitude",
			&parameters.goalLongitude) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner goalLongitude";
	}

	if (pGlobalPlanner->QueryDoubleAttribute("startLatitude",
			&parameters.startLatitude) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner startLatitude";
	}

	if (pGlobalPlanner->QueryDoubleAttribute("startLongitude",
			&parameters.startLongitude) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner startLongitude";
	}

	printf("positionEstimation -- runThread: %d\n", parameters.runThread);
	printf("positionEstimation -- debug: %d\n", parameters.debug);
	printf("positionEstimation -- encoderTicksPerRev : %d\n",
			parameters.encoderTicksPerRev);
	printf("positionEstimation -- wheelDiameter : %f\n",
			parameters.wheelDiameter);
	printf("positionEstimation -- wheelBase : %f\n", parameters.wheelBase);

	printf("positionEstimation -- predictionVariance : %f\n",
			parameters.predictionVariance);
	printf("positionEstimation -- gpsVariance : %f\n", parameters.gpsVariance);
	printf("positionEstimation -- imuVariance : %f\n", parameters.imuVariance);
	printf("positionEstimation -- encoderVariance : %f\n",
			parameters.encoderVariance);
	printf("positionEstimation -- start lat/lon : %f/%f\n",
			parameters.startLatitude, parameters.startLongitude);
	printf("positionEstimation -- goal lat/lon : %f/%f\n",
			parameters.goalLatitude, parameters.goalLongitude);
}

void PositionEstimation::run() {
	try {
		while (!gps.isOpen() && parameters.runThread)
			usleep(200);
		while ((gps.getFixStatus() == 1 || (fabs(gps.getLat()) < 0.00001)
				|| (fabs(gps.getLon()) < 0.000001)) && parameters.runThread) {
			usleep(200);
		};

		while (!gps.isDataValid() && parameters.runThread) {
			usleep(200);
		}

		if (parameters.runThread) {
			gps.setZeroXY(gps.getLat(), gps.getLon());

			logStream
					<< "Log of position estimation: timestamp (in ms), X[m], Y[m], v[m/s], theta[rad]"
					<< std::endl;

			logGpxStream << "<gpx version=\"1.0\">" << std::endl;
			logGpxStream << std::fixed << std::setprecision(10) << "<wpt lat=\""
					<< parameters.startLatitude << "\" lon=\""
					<< parameters.startLongitude
					<< "\"><name>Start</name></wpt>" << std::endl;
			logGpxStream << std::fixed << std::setprecision(10) << "<wpt lat=\""
					<< parameters.goalLatitude << "\" lon=\""
					<< parameters.goalLongitude << "\"><name>Goal</name></wpt>"
					<< std::endl;
			logGpxStream << "<trk><trkseg>" << std::endl;

		}

		struct timeval start, end;
		while (parameters.runThread) {
			gettimeofday(&start, NULL);

			// Perform EKF
			KalmanLoop();

			// Save trajectory to file
			saveTrajectoryToFile();

			// Thread sleep, so that the position is not updated too often
			// Right now 1 ms as Robot Drive has it's own sleep
			std::chrono::milliseconds duration(
					int(1000.0 / parameters.processingFrequency));
			std::this_thread::sleep_for(duration);

			gettimeofday(&end, NULL);

			long seconds = end.tv_sec - start.tv_sec;
			long useconds = end.tv_usec - start.tv_usec;
			long mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;

			if (mtime == 0)
				mtime = 1;
			if (parameters.debug == 1) {
				printf("PE: X:%5.5f \tY:%5.5f \tS:%5.5f \tA:%5.5f\n",
						state.at<double>(0), state.at<double>(1),
						state.at<double>(2), state.at<double>(3));
			}
			//cout << "PE:: framerate: " << 1000.0 / mtime << endl;
		}
	} catch (char const* error) {
		cout << "Char exception in PositionEstimation " << error << endl;
		exit(1);
	}
	catch(std::exception& e){
		cout << "Std exception in PositionEstimation: " << e.what() << endl;
		exit(1);
	}
	catch(...){
		cout << "Unexpected exception in PositionEstimation" << endl;
		exit(1);
	}
}

void PositionEstimation::stopThread() {
	parameters.runThread = false;
	if (estimationThread.joinable()) {
		estimationThread.join();
	}
}

void PositionEstimation::saveTrajectoryToFile() {
	std::chrono::milliseconds currentTime = robot->getGlobalTime();
	logStream << currentTime.count() << " " << state.at<double>(0) << " "
			<< state.at<double>(1) << " " << state.at<double>(2) << " "
			<< state.at<double>(3) << std::endl;

	double x = robot->getPosLatitude(state.at<double>(0) * 1000);
	double y = robot->getPosLongitude(state.at<double>(1) * 1000);
	x = GPS::nmea2Decimal(x) / 100;
	y = GPS::nmea2Decimal(y) / 100;

	logGpxStream << std::fixed << std::setprecision(10) << "<trkpt time=\""
			<< currentTime.count() << "\" lat=\"" << x << "\" lon=\"" << y
			<< "\"></trkpt>" << std::endl;

}

void PositionEstimation::kalmanSetup() {
	EKF = new ExtendedKalmanFilter(parameters.predictionVariance,
			parameters.gpsVariance, parameters.imuVariance,
			parameters.encoderVariance);

	state = cv::Mat(4, 1, CV_64F);
	setZeroPosition();
}

void PositionEstimation::KalmanLoop() {

	// Local variables
	std::chrono::high_resolution_clock::time_point encoderTimestamp,
			gpsTimestamp, imuTimestamp;
	bool predictPerformed = false;

	// Time difference between updates
	float predictTime =
			std::chrono::duration_cast < std::chrono::milliseconds
					> (std::chrono::high_resolution_clock::now()
							- lastUpdateTimestamp).count();
	lastUpdateTimestamp = std::chrono::high_resolution_clock::now();

	if (parameters.debug == 1) {
		printf("Kalman Loop --> predictTime: %f\n", predictTime);
	}
	// Check if GPS is online
//	printf("Checking GPS\n");
	if (isGpsOpen() && gps.getFixStatus() > 1) {
		if (parameters.debug == 1) {
			printf("GPS is online and will be used in update!\n");
		}
		// Get the GPS data if GPS is available
		gpsTimestamp = this->gps.getTimestamp();
		float gps_dt = std::chrono::duration_cast < std::chrono::milliseconds
				> (gpsTimestamp - lastGpsTimestamp).count();

		// Current measurement is newer than last update, so predict
		if (gps_dt > 0) {
			EKF->predict(predictTime / 1000);
			predictPerformed = true;

			// Correct with GPS
			lastGpsTimestamp = gpsTimestamp;
			Mat gps_data = Mat(2, 1, CV_64FC1);
			gps_data.at<double>(0, 0) = this->gps.getPosX() / 1000;
			gps_data.at<double>(1, 0) = this->gps.getPosY() / 1000;
			if (parameters.debug == 1) {
				printf("\n\n GPS : %f %f\n", gps.getLat(), gps.getLon());
				printf("GPS pos : %f %f\n", gps.getPosX() / 1000,
						gps.getPosY() / 1000);
				printf("Gps update --> values %f %f\n",
						gps_data.at<double>(0, 0), gps_data.at<double>(1, 0));
			}
			std::unique_lock < std::mutex > gpsLck(positionEstimationMtx);
			state = EKF->correctGPS(gps_data);
			gpsLck.unlock();

		}

	}

	// Check if encoder is online
//	printf("Checking encoder\n");
	if (isEncodersOpen()) {
		if (parameters.debug == 1) {
			printf("Encoders are open and will be used in update!\n");
		}
		cv::Mat enc_data = this->getEncoderData(encoderTimestamp);
		if (parameters.debug == 1) {
			printf("Encoder data: %d %d\n", enc_data.at<int>(0) - lastLeft,
					enc_data.at<int>(1) - lastRight);
		}
		float encoder_dt = std::chrono::duration_cast
				< std::chrono::milliseconds
				> (encoderTimestamp - lastEncoderTimestamp).count();
		if (encoder_dt > 0) {

			if (!predictPerformed) {
				EKF->predict(predictTime / 1000);
				predictPerformed = true;
			} else {
				EKF->predict(0);
			}

			if (parameters.debug == 1) {
				printf("Encoder data: %d %d %f\n",
						enc_data.at<int>(0) - lastLeft,
						enc_data.at<int>(1) - lastRight, encoder_dt);
			}
			lastEncoderTimestamp = encoderTimestamp;

			if (encoderStart) {
				lastLeft = enc_data.at<int>(0);
				lastRight = enc_data.at<int>(1);
				encoderStart = 0;
			}

			if (parameters.debug == 1) {
				printf("Bug test: %d %f\n", parameters.encoderTicksPerRev,
						parameters.wheelDiameter);
			}

			float left_encoder = ((float) (enc_data.at<int>(0) - lastLeft))
					/ parameters.encoderTicksPerRev * M_PI
					* parameters.wheelDiameter;
			float right_encoder = ((float) (enc_data.at<int>(1) - lastRight))
					/ parameters.encoderTicksPerRev * M_PI
					* parameters.wheelDiameter;

			if (parameters.debug == 1) {
				printf("Encoder left/right: %f %f\n", left_encoder,
						right_encoder);
			}

			float distance = (left_encoder + right_encoder) / 2.0;

			Mat speed(1, 1, CV_64FC1);
			speed.at<double>(0) = (double) (distance / encoder_dt * 1000); // Is in seconds or ms ?

			if (parameters.debug == 1) {
				printf("Encoder distance: %.10f\n", distance);
//				printf("Encoder encoder_dt: %.10f\n", encoder_dt );
				printf("Encoder speed update: %f\n", distance / encoder_dt);
			}
			std::unique_lock < std::mutex > encoderLck(positionEstimationMtx);
			state = EKF->correctEncoder(speed);
			encoderLck.unlock();

			lastLeft = enc_data.at<int>(0);
			lastRight = enc_data.at<int>(1);

		}
	}

	// Checking if info from IMU is available
//	printf("Checking IMU\n");
	if (isImuOpen()) {
		if (parameters.debug == 1) {
			printf("IMU is online and will be used\n");
		}
		cv::Mat imuData = this->imu.getData(imuTimestamp);

		if (parameters.debug == 1) {
			printf("IMU data : %f\n", imuData.at<float>(11));
		}
		float imu_dt = std::chrono::duration_cast < std::chrono::milliseconds
				> (imuTimestamp - lastImuTimestamp).count();

		// Starting imu angle
		if (imuStart) {
			imuZeroAngle = 0;
			imuStart = false;
		}

		// Imu timestamps
		if (imu_dt > 0) {
			lastImuTimestamp = imuTimestamp;

			if (!predictPerformed) {
				EKF->predict(predictTime / 1000);
				predictPerformed = true;
			} else {
				EKF->predict(0);
			}

			// 3x4 - acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
			Mat orientation(1, 1, CV_64FC1);
			orientation.at<double>(0) = (double) (imuData.at<float>(11)
					- imuZeroAngle) * M_PI / 180.0;
//			printf("IMU new update: %f\n", imuData.at<float>(11));

			std::unique_lock < std::mutex > imuLck(positionEstimationMtx);
			state = EKF->correctIMU(orientation);
			imuLck.unlock();
		}
	}
}

void PositionEstimation::setZeroPosition() {
	std::unique_lock < std::mutex > zeroLck(positionEstimationMtx);
	state.at<double>(0) = 0.0;
	state.at<double>(1) = 0.0;
	state.at<double>(2) = 0.0;
	zeroLck.unlock();

	lastLeft = 0;
	lastRight = 0;
	encoderStart = 1;
	imuStart = 1;

	if (isGpsOpen()) {
		gps.setZeroXY(gps.getLat(), gps.getLon());
	}

//	KF->statePost = KF->statePre = Mat::zeros(2,1, CV_32F);
}

bool PositionEstimation::isSetZero() {
	return gps.getIsSetZero();
}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32SC1 2x1: left, right encoder
cv::Mat PositionEstimation::getEncoderData(
		std::chrono::high_resolution_clock::time_point &timestamp) {
#ifdef TROBOT
	return robot->getEncoderData(timestamp);
#else
	return encoders.getEncoders(timestamp);
#endif
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
cv::Mat PositionEstimation::getImuData(
		std::chrono::high_resolution_clock::time_point &timestamp) {
	return imu.getData(timestamp);
}

//----------------------ACCESS TO COMPUTED DATA
//CV_64FC1 3x1: x, y, fi
const cv::Mat PositionEstimation::getEstimatedPosition() {
	std::unique_lock < std::mutex > pe(positionEstimationMtx);
	cv::Mat stateCpy = cv::Mat(3, 1, CV_64FC1);
	stateCpy = state.clone();
	pe.unlock();
	return stateCpy;
}

//----------------------MENAGMENT OF PositionEstimation DEVICES
//Gps
void PositionEstimation::openGps(std::string port) {
	gps.initController(port.c_str(), 9600);
}

void PositionEstimation::closeGps() {
	gps.deinitController();
}

bool PositionEstimation::isGpsOpen() {
	return gps.isOpen();
}

int PositionEstimation::gpsGetFixStatus() {
	return gps.getFixStatus();
}

bool PositionEstimation::isGpsDataValid() {
	return gps.isDataValid();
}

double PositionEstimation::getPosX(double longitude) {
	return gps.getPosX(longitude);
}

double PositionEstimation::getPosLongitude(double X) {
	return gps.getPosLongitude(X);
}

double PositionEstimation::getPosY(double latitude) {
	return gps.getPosY(latitude);
}

double PositionEstimation::getPosLatitude(double X) {
	return gps.getPosLatitude(X);
}

//Imu
void PositionEstimation::openImu(std::string port) {
	imu.openPort(port);
}

void PositionEstimation::closeImu() {
	imu.closePort();
}

bool PositionEstimation::isImuOpen() {
	return imu.isPortOpen();
}

bool PositionEstimation::isImuDataValid() {
	return imu.isDataValid();
}

float PositionEstimation::getImuAccVariance() {
	return imu.getAccVariance();
}

//Encoders
void PositionEstimation::openEncoders(std::string port) {
	encoders.openPort(port, 115200);
}

void PositionEstimation::closeEncoders() {
	encoders.closePort();
}

bool PositionEstimation::isEncodersOpen() {
#ifdef TROBOT
	return robot->isEncodersOpen();
#else
	return encoders.isPortOpen();
#endif
}

void PositionEstimation::fakeGPSStart(double lat, double lon) {
	gps.fakeGPSStart(lat, lon);
}

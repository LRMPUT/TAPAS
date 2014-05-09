/*
 * PositionEstimation.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
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

PositionEstimation::PositionEstimation(Robot* irobot) :
		robot(irobot), imu(irobot), gps(irobot), encoders(irobot) {

	ENCODER_TICK_PER_REV = 48 * 75;
	WHEEL_DIAMETER = 0.12;
	WHEEL_BASE = 0.24;
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
	runThread = true;
	estimationThread = std::thread(&PositionEstimation::run, this);
}

PositionEstimation::~PositionEstimation() {
	closeGps();
	closeImu();
	delete EKF;
}

void PositionEstimation::run() {

	struct timeval start, end;
	while (runThread) {
		gettimeofday(&start, NULL);

		KalmanLoop();

		// Thread sleep, so that the position is not updated too often
		// Right now 1 ms as Robot Drive has it's own sleep
		std::chrono::milliseconds duration(50);
		std::this_thread::sleep_for(duration);

		gettimeofday(&end, NULL);

		long seconds = end.tv_sec - start.tv_sec;
		long useconds = end.tv_usec - start.tv_usec;
		long mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;

		if (mtime == 0)
			mtime = 1;
		cout << "PE:: x: " << state.at<float>(0) << " y: " << state.at<float>(1)
				<< " z: " << state.at<float>(2) << endl;
		cout << "PE:: framerate: " << 1000.0 / mtime << endl;
	}
}

void PositionEstimation::stopThread() {
	runThread = false;
	estimationThread.join();
}

void PositionEstimation::kalmanSetup() {
	EKF = new ExtendedKalmanFilter();

	state = cv::Mat(4,1, CV_32F);
	setZeroPosition();
}

void PositionEstimation::KalmanLoop() {
	std::chrono::high_resolution_clock::time_point encoderTimestamp, gpsTimestamp, imuTimestamp;


	// Get the GPS data if GPS is available
	gpsTimestamp = this->gps.getTimestamp();
	if (gps.getFixStatus() > 1 && lastGpsTimestamp != gpsTimestamp) {
		lastUpdateTimestamp = gpsTimestamp;

		float dt = std::chrono::duration_cast < std::chrono::milliseconds > (lastUpdateTimestamp - gpsTimestamp).count();
		EKF->predict(dt);

		Mat gps_data = Mat(2, 1, CV_32FC1);
		gps_data.at<float>(0, 0) = this->gps.getPosX();
		gps_data.at<float>(1, 0) = this->gps.getPosY();

		state = EKF->correctGPS(gps_data);
	}

	if (isEncodersOpen()) {
		cv::Mat enc_data = this->getEncoderData(encoderTimestamp);

		if (encoderTimestamp != lastEncoderTimestamp)
		{
			lastEncoderTimestamp = encoderTimestamp;

			float dt = std::chrono::duration_cast < std::chrono::milliseconds
					> (lastUpdateTimestamp - encoderTimestamp).count();
			lastUpdateTimestamp = encoderTimestamp;
			EKF->predict(dt);

			dt = std::chrono::duration_cast < std::chrono::milliseconds
							> (lastEncoderTimestamp - encoderTimestamp).count();
			float left_encoder = ( (float) enc_data.at<int>(0)) / ENCODER_TICK_PER_REV
					* M_PI * WHEEL_DIAMETER;
			float right_encoder = ( (float) enc_data.at<int>(1)) / ENCODER_TICK_PER_REV
							* M_PI * WHEEL_DIAMETER;

			float distance = (left_encoder + right_encoder) / 2.0;

			Mat speed(1, 1, CV_32FC1);
			speed.at<float>(0) = distance / (dt / 1000); // Is in seconds or ms ?
			state = EKF->correctEncoder(speed);
		}
	}

	if (isImuOpen())
	{
		cv::Mat imuData = this->imu.getData(imuTimestamp);
		if (imuTimestamp != lastImuTimestamp)
		{
			lastImuTimestamp = imuTimestamp;

			// 3x4 - acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
			Mat orientation(1, 1, CV_32FC1);
			orientation.at<float>(0) = imuData.at<float>(11);
			state = EKF->correctIMU(orientation);
		}
	}
}


void PositionEstimation::setZeroPosition() {
	state.at<float>(0) = 0.0;
	state.at<float>(1) = 0.0;
	state.at<float>(2) = 0.0;

	if (isGpsOpen()) {
		gps.setZeroXY(gps.getLat(), gps.getLon());
	}

//	KF->statePost = KF->statePre = Mat::zeros(2,1, CV_32F);
}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32SC1 2x1: left, right encoder
cv::Mat PositionEstimation::getEncoderData(
		std::chrono::high_resolution_clock::time_point &timestamp) {
	return encoders.getEncoders(timestamp);
}

//----------------------ACCESS TO COMPUTED DATA
//CV_32SC1 3x1: x, y, fi
const cv::Mat PositionEstimation::getEstimatedPosition() {
	return state;
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

//Encoders
void PositionEstimation::openEncoders(std::string port) {
	encoders.openPort(port, 115200);
}

void PositionEstimation::closeEncoders() {
	encoders.closePort();
}

bool PositionEstimation::isEncodersOpen() {
	return encoders.isPortOpen();
}

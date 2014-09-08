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

	ENCODER_TICK_PER_REV = 300;
	WHEEL_DIAMETER = 178/1000;
	WHEEL_BASE = 432/1000;
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
		std::chrono::milliseconds duration(500);
		std::this_thread::sleep_for(duration);

		gettimeofday(&end, NULL);

		long seconds = end.tv_sec - start.tv_sec;
		long useconds = end.tv_usec - start.tv_usec;
		long mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;

		if (mtime == 0)
			mtime = 1;
		printf("PE: X:%5.5f \tY:%5.5f \tS:%5.5f \tA:%5.5f\n", state.at<float>(0), state.at<float>(1), state.at<float>(2), state.at<float>(3));

		//cout << "PE:: framerate: " << 1000.0 / mtime << endl;
	}
}

void PositionEstimation::stopThread() {
	runThread = false;
	estimationThread.join();
}

void PositionEstimation::kalmanSetup() {
	EKF = new ExtendedKalmanFilter();

	state = cv::Mat(4, 1, CV_32F);
	setZeroPosition();
}

void PositionEstimation::KalmanLoop() {
	std::chrono::high_resolution_clock::time_point encoderTimestamp,
			gpsTimestamp, imuTimestamp;

	// Get the GPS data if GPS is available
	gpsTimestamp = this->gps.getTimestamp();
	float gps_dt = std::chrono::duration_cast < std::chrono::milliseconds
			> (gpsTimestamp - lastGpsTimestamp).count();

	if (gps.getFixStatus() > 1 && gps_dt > 0) {
		float dt = std::chrono::duration_cast < std::chrono::milliseconds
				> (gpsTimestamp - lastUpdateTimestamp).count();
		if (dt > 0) {
			EKF->predict(dt);
			lastUpdateTimestamp = gpsTimestamp;
		}
		lastGpsTimestamp = gpsTimestamp;
		Mat gps_data = Mat(2, 1, CV_32FC1);
		gps_data.at<float>(0, 0) = this->gps.getPosX();
		gps_data.at<float>(1, 0) = this->gps.getPosY();

		state = EKF->correctGPS(gps_data);

	}

	if (isEncodersOpen()) {
		cv::Mat enc_data = this->getEncoderData(encoderTimestamp);

		float encoder_dt = std::chrono::duration_cast
				< std::chrono::milliseconds
				> (encoderTimestamp - lastEncoderTimestamp).count();
		if (encoder_dt > 0) {
			float dt = std::chrono::duration_cast < std::chrono::milliseconds
					> (encoderTimestamp - lastUpdateTimestamp).count();
			if (dt > 0) {
				lastUpdateTimestamp = encoderTimestamp;
				EKF->predict(dt/1000);
			}


			if (encoder_dt > 0) {
//				printf("Encoder data: %d %d\n", enc_data.at<int>(0) - lastLeft, enc_data.at<int>(1) - lastRight );
				lastEncoderTimestamp = encoderTimestamp;

				if (encoderStart)
				{
					lastLeft = enc_data.at<int>(0);
					lastRight = enc_data.at<int>(1);
					encoderStart = 0;
				}

				printf("Enc compare %d %d %d %d\n", enc_data.at<int>(0), lastLeft,
						enc_data.at<int>(1), lastRight);
				float left_encoder = ((float) (enc_data.at<int>(0) - lastLeft))
						/ ENCODER_TICK_PER_REV * M_PI * WHEEL_DIAMETER;
				float right_encoder = ((float) (enc_data.at<int>(1) - lastRight))
						/ ENCODER_TICK_PER_REV * M_PI * WHEEL_DIAMETER;

				float distance = (left_encoder + right_encoder) / 2.0;

				Mat speed(1, 1, CV_32FC1);
				speed.at<float>(0) = distance / encoder_dt * 1000 ; // Is in seconds or ms ?
//				printf("Encoder distance: %.10f\n", distance );
//				printf("Encoder encoder_dt: %.10f\n", encoder_dt );
//				printf("Encoder speed update: %f\n", distance / encoder_dt );
				state = EKF->correctEncoder(speed);

				lastLeft = enc_data.at<int>(0);
				lastRight = enc_data.at<int>(1);
			}
		}
	}

	if (isImuOpen()) {
		cv::Mat imuData = this->imu.getData(imuTimestamp);

		float imu_dt = std::chrono::duration_cast
						< std::chrono::milliseconds
						> (imuTimestamp - lastImuTimestamp).count();
//		printf( "imu dt = %f\n", imu_dt);
		if (imu_dt > 0) {
			lastImuTimestamp = imuTimestamp;

			float dt = std::chrono::duration_cast < std::chrono::milliseconds
					> (imuTimestamp - lastUpdateTimestamp).count();
			if ( dt > 0)
			{
				lastUpdateTimestamp = imuTimestamp;
				EKF->predict(dt/1000);
			}

			// 3x4 - acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
			Mat orientation(1, 1, CV_32FC1);
			orientation.at<float>(0) = imuData.at<float>(11) *  M_PI / 180.0;
//			printf("IMU update: %f\n", imuData.at<float>(11));
			state = EKF->correctIMU(orientation);
		}
	}
}

void PositionEstimation::setZeroPosition() {
	state.at<float>(0) = 0.0;
	state.at<float>(1) = 0.0;
	state.at<float>(2) = 0.0;

	lastLeft = 0;
	lastRight = 0;
	encoderStart = 1;

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
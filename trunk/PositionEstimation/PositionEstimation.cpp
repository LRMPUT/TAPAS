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

PositionEstimation::PositionEstimation(Robot* irobot) : robot(irobot) {

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

	runThread = true;
	estimationThread = std::thread(&PositionEstimation::run, this);
}

PositionEstimation::~PositionEstimation() {
	closeGps();
	closeImu();
	delete KF;
}

void PositionEstimation::run() {

	struct timeval start, end;
	while (runThread) {
		gettimeofday(&start, NULL);

		KalmanPredict();
		KalmanUpdate();

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
		//cout << "PE:: x: " << state.at<float>(0) << " y: " << state.at<float>(1)
		//		<< " z: " << state.at<float>(2) << endl;
		//cout << "PE:: framerate: " << 1000.0 / mtime << endl;
	}
}

void PositionEstimation::stopThread() {
	runThread = false;
	estimationThread.join();
}

void PositionEstimation::kalmanSetup() {
	KF = new KalmanFilter();
	KF->init(2, 2, 2);

	KF->transitionMatrix = *(Mat_<double>(2, 2) << 1, 0, 0, 1);

	KF->controlMatrix = *(Mat_<double>(2, 2) << 1, 0, 0, 1);

	KF->measurementMatrix = *(Mat_<double>(2, 2) << 1, 0, 0, 1);

	setIdentity(KF->processNoiseCov, Scalar::all(.005));
	setIdentity(KF->measurementNoiseCov, Scalar::all(0.5));

	state = cv::Mat(3,1, CV_32F);

	setZeroPosition();
}


// Update Kalman - updates on GPS
void PositionEstimation::KalmanUpdate()
{
	// Get the GPS data if GPS is available
	if ( this->robot->isGpsOpen() )
	{
		Mat gps_data = Mat(2,1, CV_32FC1);
		gps_data.at<float>(0,0) = this->gps.getPosX();
		gps_data.at<float>(1,0) = this->gps.getPosY();

		cv::Mat correctedPosition = KF->correct(gps_data);
		state.at<float>(0) = correctedPosition.at<float>(0);
		state.at<float>(0) = correctedPosition.at<float>(0);
	}
	// there was no update, so we need to copy the data
	else
	{
		KF->statePre.copyTo(KF->statePost);
		KF->errorCovPre.copyTo(KF->errorCovPost);
	}
}

// Encoders - predict
void PositionEstimation::KalmanPredict()
{
	// If the encoders are working
	if (this->robot->isRobotsDriveOpen()) {
		cv::Mat enc = this->robot->getEncoderData();

		// Getting the encoder ticks
		float left_enc = enc.at<float>(0, 0)
				/ ENCODER_TICK_PER_REV* WHEEL_DIAMETER * M_PI;
		float right_enc = enc.at<float>(1, 0)
				/ ENCODER_TICK_PER_REV* WHEEL_DIAMETER * M_PI;

		double theta = state.at<float>(2);
		// if there is IMU, we use IMU to estimate theta !:)
		if (this->isImuOpen()) {
			// Getting the angle theta of the IMU
			cv::Mat imu = this->imu.getData();

			//3x4 - acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
			theta = imu.at<float>(0, 3);
		}
		// No IMU :(
		else {
			theta += (left_enc - right_enc) / WHEEL_BASE;
		}

		double distance_covered = (left_enc + right_enc) / 2;
		cv::Mat prediction = cv::Mat(2, 1, CV_32FC1);
		prediction.at<float>(0) = distance_covered * sin(theta);
		prediction.at<float>(1) = distance_covered * cos(theta);

		cv::Mat predictedState = KF->predict(prediction);
		state.at<float>(0) = predictedState.at<float>(0);
		state.at<float>(1) = predictedState.at<float>(2);
		state.at<float>(2) = theta;
	}
	// No encoders
	else
	{
		KF->statePost.copyTo(KF->statePre);
		KF->errorCovPost.copyTo(KF->errorCovPre);
	}
}

void PositionEstimation::setZeroPosition() {
	state.at<float>(0) = 0.0;
	state.at<float>(1) = 0.0;
	state.at<float>(2) = 0.0;

	if ( isGpsOpen() )
	{
		gps.setZeroXY(gps.getLat(), gps.getLon());
	}

	KF->statePost = KF->statePre = Mat::zeros(2,1, CV_32F);
}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32SC1 2x1: left, right encoder
cv::Mat PositionEstimation::getEncoderData(){
	return encoders.getEncoders();
}

//----------------------ACCESS TO COMPUTED DATA
//CV_32SC1 3x1: x, y, fi
const cv::Mat PositionEstimation::getEstimatedPosition(){
	return state;
}

//----------------------MENAGMENT OF PositionEstimation DEVICES
//Gps
void PositionEstimation::openGps(std::string port){
	gps.initController(port.c_str(), 9600);
}

void PositionEstimation::closeGps(){
	gps.deinitController();
}

bool PositionEstimation::isGpsOpen(){
	return gps.isOpen();
}

//Imu
void PositionEstimation::openImu(std::string port){
	imu.openPort(port);
}

void PositionEstimation::closeImu(){
	imu.closePort();
}

bool PositionEstimation::isImuOpen(){
	return imu.isPortOpen();
}

//Encoders
void PositionEstimation::openEncoders(std::string port){
	encoders.openPort(port, 115200);
}

void PositionEstimation::closeEncoders(){
	encoders.closePort();
}

bool PositionEstimation::isEncodersOpen(){
	return encoders.isPortOpen();
}

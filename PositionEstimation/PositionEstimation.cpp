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

using namespace cv;
using namespace std;

PositionEstimation::PositionEstimation(Robot* irobot) : robot(irobot) {

	runThread = true;
	estimationThread = std::thread(&PositionEstimation::run, this);

	KF = new KalmanFilter();
	KF->init(2,2,2);

	/* KALMAN:
	 * - we track 2 values -> global position
	 *
	 * - to predict we can use values from encoders
	 * - to correct we can use information from the GPS
	 *
	 */

	KF->transitionMatrix =
			*(Mat_<double>(2, 2) << 1, 0,
									0, 1);

	KF->controlMatrix =
			*(Mat_<double>(2, 2) << 1, 0,
									0, 1);

	KF->measurementMatrix =
			*(Mat_<double>(2, 2) << 1, 0,
									0, 1);

}

PositionEstimation::~PositionEstimation() {
	closeGps();
	closeImu();
}

void PositionEstimation::run() {

	while(runThread)
	{
		cout<< " 1 " << endl;
	}
}

void PositionEstimation::stopThread() {
	runThread = false;
	estimationThread.join();
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
	}
}

// Encoders - predict
void PositionEstimation::KalmanPredict()
{
	double theta = 0.0;

	// Getting the encoder ticks
	float left_enc = float( this->encoders.getLeftEncoder() ) / ENCODER_TICK_PER_REV * WHEEL_DIAMETER * M_PI;
	float right_enc = float ( this->encoders.getRightEncoder() ) / ENCODER_TICK_PER_REV * WHEEL_DIAMETER * M_PI;

	// if there is IMU, we use IMU to estimate theta !:)
	if (this->isImuOpen())
	{
		// Getting the angle theta of the IMU
		cv::Mat imu = this->imu.getData();

		//acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
		theta = imu.at<float>(0,3);
	}
	// No IMU :(
	else
	{
		double prev_theta = (left_enc - right_enc) / WHEEL_BASE;
	}


	double distance_covered = (left_enc + right_enc)/2;
	cv::Mat prediction = cv::Mat(2,1, CV_32FC1);
	prediction.at<float>(0) = distance_covered * sin(theta);
	prediction.at<float>(1) = distance_covered * cos(theta);

	state = KF->predict(prediction);
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
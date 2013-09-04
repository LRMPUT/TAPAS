/*
 * PositionEstimation.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

#include <opencv2/opencv.hpp>
#include <string>
#include "PositionEstimation.h"

using namespace cv;
using namespace std;

PositionEstimation::PositionEstimation(Robot* irobot) : robot(irobot) {

	KF = new KalmanFilter();
	KF->init(6,2,3);

	float dt = 0.1; // Right now hard-coded -> cannot be like that    


	/* KALMAN:
	 * - we track 6 values -> x, y, phi, vx, vy, omega_phi
	 *
	 * - to predict we can use values from encoders
	 * - to correct we can use information from the GPS
	 *
	 */

	KF->transitionMatrix =
			*(Mat_<double>(6, 6) << 1, 0, 0, dt, 0, 0,
									0, 1, 0, 0, dt, 1,
									0, 0, 1, 0, 0, dt,
									0, 0, 0, 1, 0, 0,
									0, 0, 0, 0, 1, 0,
									0, 0 ,0, 0, 0, 1);

	KF->controlMatrix =
			*(Mat_<double>(6, 3) << 1, 0, 0,
									0, 1, 0,
									0, 0, 1,
									0, 0, 0,
									0, 0, 0,
									0, 0 ,0);

	KF->measurementMatrix =
			*(Mat_<double>(6, 2) << 1, 0,
									0, 1,
									0, 0,
									0, 0,
									0, 0,
									0, 0);

}

PositionEstimation::~PositionEstimation() {
	closeGps();
	closeImu();
}

// Update Kalman - updates on GPS
void PositionEstimation::KalmanUpdate()
{
	state = KF->correct( this->getGpsData() );
	
	// Optional stuff
	//cv::Mat imu = this->getImuData();
	// Change measurementMatrix
	// 
	// Add magneto
}

// Encoders - predict
void PositionEstimation::KalmanPredict()
{
	// TODO:
	// - move encoder TICK somewhere up
	// - what is wheel base
	int TICK_PER_ROUND = 1000;
	int WHEEL_BASE = 100;
	double PI = 3.1415265;
	int wheel_radius = 10;

	cv::Mat pred = this->robot->getEncoderData();
	pred.at<float>(0) *= pred.at<float>(0) / TICK_PER_ROUND * 2 * PI * wheel_radius;
	pred.at<float>(1) *= pred.at<float>(1) / TICK_PER_ROUND * 2 * PI * wheel_radius;
	pred.at<float>(2) = ( pred.at<float>(1) - pred.at<float>(0) ) / TICK_PER_ROUND * 2 * PI * wheel_radius / ( 2 * PI * WHEEL_BASE ) * 360;

	state = KF->predict(pred);
}

//----------------------ACCESS TO COMPUTED DATA
//CV_32SC1 3x1: x, y, fi
const cv::Mat PositionEstimation::getEstimatedPosition(){
	return state;
}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32FC1 4x1: x, y, lat, lon position
const cv::Mat PositionEstimation::getGpsData(){
	Mat ret(4, 1, CV_32FC1);
	ret.at<double>(0) = (double)gps.getPosX();
	ret.at<double>(1) = (double)gps.getPosY();
	ret.at<double>(2) = gps.getLat();
	ret.at<double>(3) = gps.getLon();
	//cout << "Position = (" << ret.at<double>(0) << ", " << ret.at<double>(1) << ")" << endl;
	return ret;
}
//1 - no fix, 2 - 2D, 3 - 3D
int PositionEstimation::getGpsFixStatus(){
	return gps.getFixStatus();
}

int PositionEstimation::getGpsSatelitesUsed(){
	return gps.getSatelitesUsed();
}

void PositionEstimation::setGpsZeroPoint(double lat, double lon){
	gps.setZeroXY(lat, lon);
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
const cv::Mat PositionEstimation::getImuData(){
	return imu.getData();
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

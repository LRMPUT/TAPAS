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

PositionEstimation::PositionEstimation(Robot* irobot) : robot(irobot), KF(6, 2, 3) {

	float dt = 0.1; // Right now hard-coded -> cannot be like that    
	float transData[]  = {  1, 0, 0, dt, 0, 0,
            				0, 1, 0, 0, dt, 1,
            				0, 0, 1, 0, 0, dt,
							0, 0, 0, 1, 0, 0,
							0, 0, 0, 0, 1, 0,
							0, 0 ,0, 0, 0, 1 };
	//memcpy( KF.transitionMatrix.data.fl, transData, sizeof(transData));

	float contrData[]  = {  1, 0, 0,
            				0, 1, 0,
            				0, 0, 1,
							0, 0, 0,
							0, 0, 0,
							0, 0 ,0 };
	//memcpy( KF.controlMatrix->data.fl, contrData, sizeof(contrData));

	float measData[]  = {   1, 0,
            				0, 1,
            				0, 0,
							0, 0,
							0, 0,
							0, 0 };
	//memcpy( KF.measurementMatrix->data.fl, measData, sizeof(measData));

}

PositionEstimation::~PositionEstimation() {
	closeGps();
	closeImu();
}

// Update Kalman - updates on GPS
void PositionEstimation::KalmanUpdate()
{
	state = KF.correct( this->getGpsData() );
	
	// Optional stuff
	//cv::Mat imu = this->getImuData();
	// Change measurementMatrix
	// 
	// Add magneto
}

// Encoders - predict
void PositionEstimation::KalmanPredict()
{
	//cv::Mat pred = this.robot->globalPlanner.getEncoderData;
	// pred.at<float>(0) *= pred.at<float>(0) / TICK_PER_ROUND * 2 * PI * wheel_radius 
	// pred.at<float>(1) *= pred.at<float>(1) / TICK_PER_ROUND * 2 * PI * wheel_radius 
	// pred.at<float>(2) = ( pred.at<float>(1) - pred.at<float>(0) ) / TICK_PER_ROUND * 2 * PI * wheel_radius / ( 2 * PI * WHEEL_BASE ) * 360 
	//
	//state = KF.predict(pred);
}

//----------------------ACCESS TO COMPUTED DATA
//CV_32SC1 3x1: x, y, fi
cv::Mat PositionEstimation::getEstimatedPosition(){
	return state;
}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32SC1 2x1: x, y position
cv::Mat PositionEstimation::getGpsData(){
	Mat ret(2, 1, CV_32SC1);
	ret.at<int>(0) = (int)gps.getPosX();
	ret.at<int>(1) = (int)gps.getPosY();
	return ret;
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
cv::Mat PositionEstimation::getImuData(){
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

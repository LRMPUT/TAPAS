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

}

PositionEstimation::~PositionEstimation() {
	closeGps();
	closeImu();
}

//----------------------ACCESS TO COMPUTED DATA
//CV_32SC1 3x1: x, y, fi
cv::Mat PositionEstimation::getEstimatedPosition(){
	return Mat(3, 1, CV_32SC1);
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

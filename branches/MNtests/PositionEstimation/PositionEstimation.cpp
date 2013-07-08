/*
 * PositionEstimation.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

#include "PositionEstimation.h"

using namespace cv;
using namespace std;

/*PositionEstimation::PositionEstimation(Robot* irobot) : robot(irobot) {

}*/

PositionEstimation::~PositionEstimation() {
	closeGps();
	closeImu();
}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32SC1 2x1: x, y position
cv::Mat PositionEstimation::getGpsData(){
	Mat ret(2, 1, CV_32SC1);
	ret.at<int32_t>(0) = (int32_t)gps.getPosX();
	ret.at<int32_t>(1) = (int32_t)gps.getPosY();
	return ret;
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
cv::Mat PositionEstimation::getImuData(){
	return imu.getData();
}

//----------------------MENAGMENT OF PositionEstimation DEVICES
//Gps
void PositionEstimation::openGps(std::string port){
	// This is madness!!!
	// Either string or char*, but char* is reaaallly deprecated
	char *cstr = new char[port.length() + 1];
	strcpy(cstr, port.c_str());
	gps.initController( cstr , 9600);
    delete [] cstr;
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

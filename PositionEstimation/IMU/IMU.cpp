/*
 * IMU.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include "IMU.h"
#include "../../Trobot/include/Imu.h"

using namespace cv;
using namespace trobot;
using namespace std;

Quantity quantities[NUM_VALUES] = {
	Quantity(4*trobot::ACCEL_PROC_XY + 2,	0.000183105),
	Quantity(4*trobot::ACCEL_PROC_XY,		0.000183105),
	Quantity(4*trobot::ACCEL_PROC_Z + 2,	0.000183105),
	Quantity(4*trobot::GYRO_PROC_XY + 2,	0.0610352),
	Quantity(4*trobot::GYRO_PROC_XY,		0.0610352),
	Quantity(4*trobot::GYRO_PROC_Z + 2,	0.0610352),
	Quantity(4*trobot::MAG_PROC_XY + 2,	0.000305176),
	Quantity(4*trobot::MAG_PROC_XY,		0.000305176),
	Quantity(4*trobot::MAG_PROC_Z + 2,		0.000305176),
	Quantity(4*trobot::EULER_PHI_THETA + 2,0.0109863),
	Quantity(4*trobot::EULER_PHI_THETA,	0.0109863),
	Quantity(4*trobot::EULER_PSI,			0.0109863)
};

IMU::IMU() : imu(NULL) {

}

IMU::~IMU() {
	closePort();
}

void IMU::openPort(std::string port){
	imu = new Imu(115200, port);
}

void IMU::closePort(){
	if(imu != NULL){
		delete imu;
		imu = NULL;
	}
}

bool IMU::isPortOpen(){
	return (imu != NULL);
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
const cv::Mat IMU::getData(){
	Mat ret(3, 4, CV_32FC1);
	for(int i = 0; i < NUM_VALUES; i++){
		float tmp = (short)((imu->Register[quantities[i].address / 4]._int >> 8*(quantities[i].address % 4)) & 0xffff);
		tmp *= quantities[i].factor;
		ret.at<float>(i % 3, i / 3) = tmp;
	}
	return ret;
}

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
	Quantity(4*trobot::EULER_PSI + 2,			0.0109863)
};

IMU::IMU() : imu(NULL), imuNew(NULL), usedIMUType(IMU_MICROSTRAIN_GX4_25) {
}

IMU::IMU(Robot* irobot) : imu(NULL), imuNew(NULL), robot(irobot), usedIMUType(IMU_MICROSTRAIN_GX4_25) {

}

IMU::~IMU() {
	closePort();
}

void IMU::openPort(std::string port){
	if ( usedIMUType == IMU_UM6)
	{
		imu = new Imu(115200, port);
	}
	else if ( usedIMUType == IMU_MICROSTRAIN_GX4_25)
	{
		imuNew = new IMU_driver();
		imuNew->openPort(port);
	}
}

void IMU::closePort(){
	if(imu != NULL){
		if ( usedIMUType == IMU_UM6)
			delete imu;
		else if ( usedIMUType == IMU_MICROSTRAIN_GX4_25)
			delete imuNew;
		imu = NULL;
	}
}

bool IMU::isPortOpen(){
	return (imu != NULL) || (imuNew != NULL);
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
cv::Mat IMU::getUM6Data(std::chrono::high_resolution_clock::time_point &timestamp){
	Mat ret(3, 4, CV_32FC1);
	for(int i = 0; i < NUM_VALUES; i++){
		float tmp = (short)((imu->Register[quantities[i].address / 4]._int >> 8*(quantities[i].address % 4)) & 0xffff);
		tmp *= quantities[i].factor;
		//uint32 tmp = (imu->Register[quantities[i].address / 4]._int);
		//tmp >>= 8*(quantities[i].address % 4);
		//tmp &= 0xffff;

		//cout << (short)tmp*quantities[i].factor << " ";
		ret.at<float>(i % 3, i / 3) = tmp;
		//cout << ret.at<float>(i % 3, i / 3) << " ";
	}
	//cout << endl;
	timestamp = imu->getTimestamp();
	return ret;
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
cv::Mat IMU::getGX4Data(std::chrono::high_resolution_clock::time_point &timestamp){
	Mat ret(3, 4, CV_32FC1);

	float* acc = imuNew->getAccel();
	float* gyro = imuNew->getGyro();
	float* mag = imuNew->getMag();
	float* euler = imuNew->getEuler();

	for(int i = 0; i < 3; i++){
		ret.at<float>(i, 0) = acc[0];
		ret.at<float>(i, 1) = gyro[0];
		ret.at<float>(i, 2) = mag[0];
		ret.at<float>(i, 3) = euler[0];
	}
	timestamp = imuNew->getTimestamp();
	return ret;
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
cv::Mat IMU::getData(std::chrono::high_resolution_clock::time_point &timestamp){
	if ( usedIMUType == IMU_UM6)
		return getUM6Data(timestamp);
	else if ( usedIMUType == IMU_MICROSTRAIN_GX4_25)
		return getGX4Data(timestamp);
	return cv::Mat(3, 4, CV_32FC1);
}

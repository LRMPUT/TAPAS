/*
	Copyright (c) 2014,	TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
	Poznan University of Technology
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice,
	this list of conditions and the following disclaimer in the documentation
	and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
	THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
	AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include "IMU.h"
#ifdef TROBOT
#include "../../Trobot/include/Imu.h"
using namespace trobot;
#endif

using namespace cv;
using namespace std;

#ifdef TROBOT
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
#endif

IMU::IMU() :
#ifdef TROBOT
	imu(NULL),
#endif
	imuNew(NULL),
	usedIMUType(IMU_MICROSTRAIN_GX4_25)
{

}

IMU::IMU(Robot* irobot) :
#ifdef TROBOT
		imu(NULL),
#endif
		imuNew(NULL),
		robot(irobot),
		usedIMUType(IMU_UM6)
{

}

IMU::~IMU() {
	closePort();
}

void IMU::openPort(std::string port){
	if ( usedIMUType == IMU_UM6)
	{
#ifdef TROBOT
		imu = new Imu(115200, port);
#endif
	}
	else if ( usedIMUType == IMU_MICROSTRAIN_GX4_25)
	{
		imuNew = new IMU_driver();
		imuNew->openPort(port);
	}
	std::cout<<"IMU opened"<<std::endl;
}

void IMU::closePort(){
	std::cout<<"IMU::closePort"<<std::endl;
	if ( usedIMUType == IMU_UM6){
#ifdef TROBOT
		if(imu != NULL){
			delete imu;
			imu = NULL;
		}
#endif
	}
	else if ( usedIMUType == IMU_MICROSTRAIN_GX4_25){
		if(imuNew != NULL){
			delete imuNew;
			imuNew = NULL;
		}
	}
	std::cout<<"End IMU::closePort"<<std::endl;
}

bool IMU::isPortOpen(){
#ifdef TROBOT
	return (imu != NULL) || (imuNew != NULL);
#else
	return (imuNew != NULL);
#endif
}


bool IMU::isDataValid(){
	if ( usedIMUType == IMU_UM6){
#ifdef TROBOT
		if(imu == NULL){
			return false;
		}
		return imu->isEulerDataValid();
#else
		return false;
#endif
	}
	else if ( usedIMUType == IMU_MICROSTRAIN_GX4_25){
		if (imuNew==NULL)
			return false;
		return imuNew->isDataValid();
	}
	return false;
}

float IMU::getAccVariance(){
	if ( usedIMUType == IMU_UM6){
//		throw "IMU::getAccVariance() not supported";
		return 0.0;
	}
	else if ( usedIMUType == IMU_MICROSTRAIN_GX4_25){
		if (imuNew==NULL)
			return -1;
		return imuNew->getAccVariance();
	}
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
cv::Mat IMU::getUM6Data(std::chrono::high_resolution_clock::time_point &timestamp){

	Mat ret(3, 4, CV_32FC1);
#ifdef TROBOT
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
#endif
	return ret;
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
cv::Mat IMU::getGX4Data(std::chrono::high_resolution_clock::time_point &timestamp){
	Mat ret(3, 4, CV_32FC1);

	float *acc, *gyro, *mag, *euler;
	acc = new float[3];
	gyro = new float[3];
	mag = new float[3];
	euler = new float[3];

	imuNew->getAccel(acc);
	imuNew->getGyro(gyro);
	imuNew->getMag(mag);
	imuNew->getEuler(euler);

	for(int i = 0; i < 3; i++){
		ret.at<float>(i, 0) = acc[i];
		ret.at<float>(i, 1) = gyro[i];
		ret.at<float>(i, 2) = mag[i];
		ret.at<float>(i, 3) = euler[i];
	}

	delete []acc;
	delete []gyro;
	delete []mag;
	delete []euler;

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

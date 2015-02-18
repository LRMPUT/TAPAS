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

#ifndef ROBOT_H_
#define ROBOT_H_

//STL
#include <fstream>
#include <string>
//OpenCV
#include <opencv2/opencv.hpp>
//TinyXML
#include <tinyxml.h>
//Boost
#include <boost/filesystem.hpp>

// Time measurement
#include <chrono>

class Robot;
class Debug;

//RobotsIntellect
#include "../PositionEstimation/PositionEstimation.h"
#include "../MovementConstraints/MovementConstraints.h"
#include "../Planning/GlobalPlanner.h"
//TinyXML
#include <tinyxml.h>


#define MAP_RASTER_SIZE 200		//[mm]
#define MAP_SIZE	(10000/MAP_RASTER_SIZE)	//[u] 10m
#define MAP_MARGIN (2500/MAP_RASTER_SIZE)	//[u] 2,5m margin

//#define NO_CUDA
//#define ROBOT_OFFLINE

#define PI 3.14159265359

class Robot {

	friend class Debug;

	void readPositionEstimationSettings(TiXmlElement* settings);



	// Class containing information about our position estimation from sensors
	PositionEstimation* positionEstimation;

	// Class responsible for planning
	GlobalPlanner* globalPlanner;

	MovementConstraints* movementConstraints;

	// Global time of start
	std::chrono::high_resolution_clock::time_point startTime;

public:
	Robot(boost::filesystem::path settings);
	virtual ~Robot();

	// Get global time
	std::chrono::milliseconds getGlobalTime() const {
		return std::chrono::duration_cast < std::chrono::milliseconds
				> (std::chrono::high_resolution_clock::now() - startTime);
	}

	// Start the program
	void startCompetition();

	//----------------------MENAGMENT OF GlobalPlanner DEVICES
	//Robots Drive
	void openRobotsDrive(std::string port1, std::string port2);

	void closeRobotsDrive();

	bool isRobotsDriveOpen();

	//----------------------MENAGMENT OF PositionEstimation DEVICES
	//Gps
	void openGps(std::string port);

	void closeGps();

	bool isGpsOpen();

	bool isSetZero();

	int gpsGetFixStatus();

	bool isGpsDataValid();

	double getPosX(double latitude);
	double getPosLatitude(double X);

	double getPosY(double longitude);
	double getPosLongitude(double Y);

	void fakeGPSStart(double lat, double lon);

	//Imu
	void openImu(std::string port);
	void closeImu();
	bool isImuOpen();
	bool isImuDataValid();

	float getImuAccVariance();

	//Encoders
	void openEncoders(std::string port);

	void closeEncoders();

	bool isEncodersOpen();

	//----------------------MENAGMENT OF MovementConstraints DEVICES
	//Hokuyo
	void openHokuyo(std::string port);

	void closeHokuyo();

	bool isHokuyoOpen();

	//Camera
	void openCamera(std::vector<std::string> device);

	void closeCamera();

	bool isCameraOpen();

	//----------------------EXTERNAL ACCESS TO MEASUREMENTS
	//CV_32SC1 2x1: left, right encoder
	cv::Mat getEncoderData();

	//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
	cv::Mat getImuData(std::chrono::high_resolution_clock::time_point &timestamp);

	//----------------------ACCESS TO COMPUTED DATA
	//CV_32SC1 3x1: x, y, fi
	const cv::Mat getEstimatedPosition();

	//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied
	cv::Mat getMovementConstraints();

	cv::Mat getPosImuConstraintsMapCenter();

	//cv::Mat getLocalMapPosInGlobalMap();

	void getLocalPlanData(cv::Mat& constraintsMap,cv::Mat& posRobotMapCenter, cv::Mat& globalMapCenter);
};

#endif /* ROBOT_H_ */

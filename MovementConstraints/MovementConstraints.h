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

#ifndef MOVEMENTCONSTRAINTS_H_
#define MOVEMENTCONSTRAINTS_H_

//STL
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <queue>
#include <functional>
//OpenCV
#include <opencv2/opencv.hpp>
//TinyXML
#include <tinyxml.h>
//RobotsIntellect
#include "Camera/Camera.h"
#include "Hokuyo/Hokuyo.h"
#include "Sharp/Sharp.h"
#include "ConstraintsHelpers.h"
#include "../Robot/RosHelpers.h"
//ROS
#include "TAPAS/PointCloud.h"
#include "TAPAS/IMU.h"
#include "TAPAS/PlanningData.h"
#include "TAPAS/Encoders.h"

class Robot;
class Debug;

class MovementConstraints {
	friend class Debug;

public:
	struct PointCloudSettings{
		float wheelCir;

		float wheelDistance;

		int encodersCPR;

		int minLaserDist;

		int mapTimeout;

		int pointCloudTimeout;
	};

private:

	// Class to get data from Hokuyo
	Hokuyo hokuyo;

//	// Class to get data from Sharp
//	Sharp sharp;

	ros::NodeHandle nh;

	ros::Subscriber imu_sub;

	ros::Subscriber encoders_sub;

	ros::ServiceClient cameraConstraintsClient;

	//CV_32FC1 4x4: camera origin position and orientation w.r.t. global coordinate system
	cv::Mat imuOrigRobot;

	//CV_32FC1 4x4: camera origin position and orientation w.r.t. laser coordinate system
	cv::Mat cameraOrigLaser;

	cv::Mat cameraOrigImu;

	float laserLowerThreshold, laserUpperThreshold;

	int laserMinPts;

	//Point Cloud from Hokuyo
	std::mutex mtxPointCloud;

	std::mutex mtxMap;

	std::mutex mtxImu;

	std::mutex mtxEncoders;

	cv::Mat curPosOrigMapCenter;

	cv::Mat curMapCenterOrigGlobal;

	cv::Mat pointCloudOrigMapCenter;

	std::chrono::high_resolution_clock::time_point timestampMap;

	//Queue of points
	std::queue<ConstraintsHelpers::PointsPacket> pointsQueue;

	bool imuActive, encodersActive;

	cv::Mat imuCur, encodersCur;

	cv::Mat imuPrev, encodersPrev;

	//
	cv::Mat constraintsMap;

	std::thread constraintsMapThread;

	bool constraintsMapThreadRunning;

	std::thread movementConstraintsThread;

	std::thread dataThread;

	bool runThread;

	int debugLevel;

	PointCloudSettings pointCloudSettings;

	// Main loop of MovementContraints thread.
	void run();

	void sendData();

	void readSettings(TiXmlElement* settings);

	cv::Mat readMatrixSettings(TiXmlElement* parent, const char* node, int rows, int cols);

	void imuCallback(const TAPAS::IMU msg);

	void encodersCallback(const TAPAS::Encoders msg);

	void updateConstraintsMap();

	void insertHokuyoConstraints(cv::Mat map,
								std::chrono::high_resolution_clock::time_point curTimestampMap,
								cv::Mat mapMove);

	void insertCameraConstraints(cv::Mat map,
								std::chrono::high_resolution_clock::time_point curTimestampMap,
								cv::Mat mapMove);

	void updateCurPosOrigMapCenter();

	void updatePointCloud();

public:
	MovementConstraints(TiXmlElement* settings);
	virtual ~MovementConstraints();

	// Stop MovementConstraints thread.
	void stopThread();

	const PointCloudSettings& getPointCloudSettings();

	//----------------------EXTERNAL ACCESS TO MEASUREMENTS
	//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
	const cv::Mat getHokuyoData();

	//----------------------ACCESS TO COMPUTED DATA
	//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
	const cv::Mat getMovementConstraints();

	bool getPointCloud(TAPAS::PointCloud::Request &req, TAPAS::PointCloud::Response &res);

	cv::Mat getPointCloud(cv::Mat& curPosMapCenter);

	cv::Mat getCurMapCenterOrigGlobal();

	cv::Mat getPosImuMapCenter();

	void getLocalPlanningData(cv::Mat& retConstraintsMap, cv::Mat& posRobotMapCenter, cv::Mat& globalMapCenter);
	//----------------------MENAGMENT OF MovementConstraints DEVICES
	//Hokuyo
	void openHokuyo(std::string port);

	void closeHokuyo();

	bool isHokuyoOpen();
};

#include "../Robot/Robot.h"

#endif /* MOVEMENTCONSTAINTS_H_ */

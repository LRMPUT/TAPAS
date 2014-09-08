/*
 * MovementConstaints.h
 *
 */

#ifndef MOVEMENTCONSTAINTS_H_
#define MOVEMENTCONSTAINTS_H_

//STL
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <queue>
//OpenCV
#include <opencv2/opencv.hpp>
//TinyXML
#include <tinyxml.h>
//RobotsIntellect
#include "Camera/Camera.h"
#include "Hokuyo/Hokuyo.h"
#include "Sharp/Sharp.h"

class Robot;
class Debug;

class MovementConstraints {
	friend class Debug;

public:
	struct PointsPacket{
		std::chrono::high_resolution_clock::time_point timestamp;
		int numPoints;
		PointsPacket(std::chrono::high_resolution_clock::time_point itimestamp,
					int inumPoints) : timestamp(itimestamp), numPoints(inumPoints)
		{}
	};

	struct PointCloudSettings{
		float wheelCir;

		float wheelDistance;

		int encodersCPR;

		int minLaserDist;

		int mapTimeout;

		int pointCloudTimeout;
	};

private:

	// Class to get data from Camera
	Camera* camera;

	// Class to get data from Hokuyo
	Hokuyo hokuyo;

	// Class to get data from Sharp
	Sharp sharp;

	//Parent class Robot
	Robot* robot;

	//CV_32FC1 4x4: camera origin position and orientation w.r.t. global coordinate system
	cv::Mat imuOrigGlobal;

	//CV_32FC1 4x4: camera origin position and orientation w.r.t. laser coordinate system
	cv::Mat cameraOrigLaser;

	cv::Mat cameraOrigImu;

	//CV_32FC1 4x1: ground plane equation [A, B, C, D]'
	cv::Mat groundPlane;

	//Point Cloud from Hokuyo
	std::mutex mtxPointCloud;

	std::mutex mtxMap;

	cv::Mat curPosCloudMapCenter;

	cv::Mat posMapCenterGlobal;

	cv::Mat pointCloudImuMapCenter;

	std::chrono::high_resolution_clock::time_point timestampMap;

	//Queue of points
	std::queue<PointsPacket> pointsQueue;

	cv::Mat imuPrev, encodersPrev;

	//
	cv::Mat constraintsMap;

	//Map center position in global coordinates
	double mapCenterX, mapCenterY, mapCenterPhi;

	std::thread movementConstraintsThread;

	bool runThread;

	PointCloudSettings pointCloudSettings;

	// Main loop of MovementContraints thread.
	void run();

	// Stop MovementConstraints thread.
	void stopThread();

	void readSettings(TiXmlElement* settings);

	cv::Mat readMatrixSettings(TiXmlElement* parent, const char* node, int rows, int cols);

	void updateConstraintsMap();

	void insertHokuyoConstraints(cv::Mat map,
								std::chrono::high_resolution_clock::time_point curTimestampMap);

	void updateCurPosCloudMapCenter();

	void updatePointCloud();

public:
	MovementConstraints(Robot* irobot, TiXmlElement* settings);
	virtual ~MovementConstraints();

	static cv::Mat compOrient(cv::Mat imuData);

	static cv::Mat compTrans(	cv::Mat orient,
								cv::Mat encodersDiff,
								const PointCloudSettings& pointCloudSettings);

	static cv::Mat compNewPos(cv::Mat lprevImu, cv::Mat lcurImu,
								cv::Mat lprevEnc, cv::Mat lcurEnc,
								cv::Mat lposMapCenter,
								cv::Mat lmapCenterGlobal,
								const PointCloudSettings& pointCloudSettings);

	static void processPointCloud(cv::Mat hokuyoData,
								cv::Mat& pointCloudImuMapCenter,
								std::queue<PointsPacket>& pointsInfo,
								std::chrono::high_resolution_clock::time_point hokuyoTimestamp,
								std::chrono::high_resolution_clock::time_point curTimestamp,
								cv::Mat curPosCloudMapCenter,
								std::mutex& mtxPointCloud,
								cv::Mat cameraOrigLaser,
								cv::Mat cameraOrigImu,
								const PointCloudSettings& pointCloudSettings);

	const PointCloudSettings& getPointCloudSettings();

	//----------------------EXTERNAL ACCESS TO MEASUREMENTS
	//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
	const cv::Mat getHokuyoData();

	//----------------------ACCESS TO COMPUTED DATA
	//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
	const cv::Mat getMovementConstraints();

	cv::Mat getPointCloud(cv::Mat& curPosMapCenter);

	cv::Mat getPosImuMapCenter();

	void getLocalPlanningData(cv::Mat& MovementConstraints, cv::Mat& PosImuMapCenter, cv::Mat& GlobalMapCenter);
	//----------------------MENAGMENT OF MovementConstraints DEVICES
	//Hokuyo
	void openHokuyo(std::string port);

	void closeHokuyo();

	bool isHokuyoOpen();

	//Camera
	void openCamera(std::vector<std::string> device);

	void closeCamera();

	bool isCameraOpen();
};

#include "../Robot/Robot.h"

#endif /* MOVEMENTCONSTAINTS_H_ */

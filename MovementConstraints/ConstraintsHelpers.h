
#ifndef CONSTRAINTSHELPERS_H_
#define CONSTRAINTSHELPERS_H_

#include <mutex>
#include <chrono>
#include <queue>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "../Robot/RosHelpers.h"

class ConstraintsHelpers {

public:
	struct PointsPacket{
		std::chrono::high_resolution_clock::time_point timestamp;
		int numPoints;
		PointsPacket(std::chrono::high_resolution_clock::time_point itimestamp,
					int inumPoints) : timestamp(itimestamp), numPoints(inumPoints)
		{}
	};

	static cv::Mat compOrient(cv::Mat imuData);

	static cv::Mat compTrans(	cv::Mat orient,
								cv::Mat encodersDiff,
								ros::NodeHandle &nh);

	static cv::Mat compNewPos(cv::Mat lprevImu, cv::Mat lcurImu,
								cv::Mat lprevEnc, cv::Mat lcurEnc,
								cv::Mat lposOrigMapCenter,
								cv::Mat lmapCenterOrigGlobal,
								ros::NodeHandle &nh);

	static void processPointCloud(cv::Mat hokuyoData,
								cv::Mat& pointCloudOrigMapCenter,
								std::queue<PointsPacket>& pointsInfo,
								std::chrono::high_resolution_clock::time_point hokuyoTimestamp,
								std::chrono::high_resolution_clock::time_point curTimestamp,
								cv::Mat curPosOrigMapCenter,
								std::mutex& mtxPointCloud,
								cv::Mat cameraOrigLaser,
								cv::Mat cameraOrigImu,
								ros::NodeHandle &nh);
};

#include "../Robot/Robot.h"

#endif /* CONSTRAINTSHELPERS_H_ */
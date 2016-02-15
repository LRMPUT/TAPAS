#ifndef PLANNERHELPERS_H_
#define PLANNERHELPERS_H_

#include <vector>
#include <opencv2/opencv.hpp>

class PlannerHelpers {

public:

	struct Parameters {
		int runThread;
		int debug;
		int avoidObstacles;
		int histResolution;
		//float threshold;
		float steeringMargin;
		float gauss3sig;
		float maxDistance;
		float backwardsPenalty;
		float normalSpeed;
		float preciseSpeed;
		float gentleTurnMargin;
		float gentleTurnSpeedDiff;
		float turnSpeed;
		int turnTimeout;
		int interruptTime;
		float interruptSpeed;
		float imuAccVarianceLimit;
	};

	static void updateHistogram(std::vector<float>& histSectors,
							cv::Mat posRobotMapCenter,
							cv::Mat constraints,
							Parameters localPlannerParams);

	static void smoothHistogram(std::vector<float>& histSectors,
								Parameters localPlannerParams);

	static float determineGoalInLocalMap(cv::Mat posLocalToGlobalMap,
										float goalDirGlobalMap);

	static float findOptimSector(const std::vector<float>& histSectors,
								cv::Mat posRobotMapCenter,
								float goalDirLocalMap,
								Parameters localPlannerParams);

	static float rotMatToEulerYaw(cv::Mat rotMat);
};

#include "GlobalPlanner.h"
#include "../Robot/Robot.h"

#endif /* PLANNERHELPERS_H_ */
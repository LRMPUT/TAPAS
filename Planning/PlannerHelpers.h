#ifndef PLANNERHELPERS_H_
#define PLANNERHELPERS_H_

class PlannerHelpers {
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

#endif /* LOCALPLANNER_H_ */
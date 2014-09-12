/*
 * LocalPlanner.cpp
 *
 *      Author: sebastian
 */

#include "LocalPlanner.h"

using namespace cv;
using namespace std;

#define LEFT_CHANNEL 2
#define RIGHT_CHANNEL 1

LocalPlanner::LocalPlanner(Robot* irobot, GlobalPlanner* planer,
		TiXmlElement* settings) :
		robot(irobot), globalPlanner(planer), startOperate(false) {
	cout << "LocalPlanner()" << endl;

	readSettings(settings);
	numHistSectors = 360/localPlannerParams.histResolution;

	localPlannerThread = std::thread(&LocalPlanner::run, this);

	cout << "End LocalPlanner()" << endl;
}

LocalPlanner::~LocalPlanner() {
	cout << "~LocalPlanner()" << endl;
	stopThread();
	cout << "End ~LocalPlanner()" << endl;
}

void LocalPlanner::readSettings(TiXmlElement* settings) {

	TiXmlElement* pLocalPlanner;
	pLocalPlanner = settings->FirstChildElement("LocalPlanner");

	if (pLocalPlanner->QueryIntAttribute("runThread", &localPlannerParams.runThread)
			!= TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for localPlanner runThread";
	}

	if (pLocalPlanner->QueryIntAttribute("runThread", &localPlannerParams.avoidObstacles)
			!= TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for localPlanner avoidObstacles";
	}

	if (pLocalPlanner->QueryIntAttribute("VFH_HistResolution",
			&localPlannerParams.histResolution) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for localPlanner VFH_HistResolution";
	}

	if (pLocalPlanner->QueryFloatAttribute("VFH_Threshold",
			&localPlannerParams.threshold) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for localPlanner VFH_Threshold";
	}
	if (pLocalPlanner->QueryFloatAttribute("VFH_SteeringMargin",
			&localPlannerParams.steeringMargin) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for VFH_SteeringMargin";
	}
	if (pLocalPlanner->QueryFloatAttribute("VFH_Gauss3sig",
			&localPlannerParams.gauss3sig) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for VFH_Gauss3sig";
	}


	printf("LocalPlanner -- runThread: %d\n", localPlannerParams.runThread);
	printf("LocalPlanner -- avoidObstacles: %d\n", localPlannerParams.avoidObstacles);
	printf("LocalPlanner -- VFH_HistResolution: %d\n",
			localPlannerParams.histResolution);
	printf("LocalPlanner -- VFH_Threshold: %f\n", localPlannerParams.threshold);
	printf("LocalPlanner -- VFH_SteeringMargin: %f\n",
			localPlannerParams.steeringMargin);
}

void LocalPlanner::startLocalPlanner() {
	startOperate = true;
}

void LocalPlanner::stopLocalPlanner() {
	cout << "LocalPlanner::stopLocalPlanner()" << endl;
	startOperate = false;
}

void LocalPlanner::run() {
	while (localPlannerParams.runThread) {
		if (startOperate) {
			executeVFH();
		}
		std::chrono::milliseconds duration(50);
		std::this_thread::sleep_for(duration);
	}
}

void LocalPlanner::executeVFH() {


	//constraint data
	Mat constraints;
	Mat posImuMapCenter;
	Mat posLocalToGlobalMap;

	robot->getLocalPlanData(constraints, posImuMapCenter,
			posLocalToGlobalMap);
	if(!constraints.empty() && !posImuMapCenter.empty() && !posLocalToGlobalMap.empty()){
		vector<float> histSectors(numHistSectors, 0);
		vector<int> freeSectors;
		if ( localPlannerParams.avoidObstacles == 1)
		{
			//cout << "updateHistogram()" << endl;
			updateHistogram(histSectors, posImuMapCenter, constraints);
			//cout << "smoothHistogram" << endl;
			smoothHistogram(histSectors);
			//cout << "findFreeSectors()" << endl;
			findFreeSectors(histSectors, freeSectors);
		}


		float goalDirGlobalMap = setGoalDirection(posLocalToGlobalMap);
		/// goal direction in the local map coordiantes
		float goalDirLocalMap = determineGoalInLocalMap(posLocalToGlobalMap, goalDirGlobalMap);

		float bestDirLocalMap = 0;
		/// optimal direction in the local map - nearest to the goal
		if ( localPlannerParams.avoidObstacles == 1)
		{
			bestDirLocalMap = findOptimSector(freeSectors, goalDirLocalMap);
		}
		else
		{
			bestDirLocalMap = goalDirLocalMap;
		}

		determineDriversCommand(posImuMapCenter,
								bestDirLocalMap);

		std::unique_lock<std::mutex> lck(mtxVecFieldHist);
		vecFieldHist = histSectors;
		shBestDirLocalMap = bestDirLocalMap;
		shGoalDirLocalMap = goalDirLocalMap;
		//cout << "vecFieldHist.size() = " << vecFieldHist.size() << endl;
		lck.unlock();
	}

}

/*float LocalPlanner::getLocalDirection() {
	return bestDirection;
}*/

float LocalPlanner::setGoalDirection(cv::Mat posLocalToGlobalMap) {

	///// this part of code is only for  test purposes
	// to test the straight ahead direction of movement:
	// logic flow: get current orientation ane keep it
	static const bool forwardRun = true;
	static bool setStraighAheadDirection = false;
	static float direction = 0;

	if(forwardRun){
		if (!setStraighAheadDirection) {
			float goalDirection = globalPlanner->getHeadingToGoal();
			//Mat posLocalToGlobalMap = robot->getLocalMapPosInGlobalMap();
			/// and convert this orentation to euler yaw
			direction = RotMatToEulerYaw(posLocalToGlobalMap);
			direction = 180 / PI * direction;
			goalDirection = direction;
			setStraighAheadDirection = true;
		}
	}
	else{
		direction = globalPlanner->getHeadingToGoal();
	}

	return direction;
}

void LocalPlanner::localPlanerTest() {
	executeVFH();
}

float LocalPlanner::RotMatToEulerYaw(Mat rotMat) {

	float R11 = rotMat.at<float>(0, 0);
	float R21 = rotMat.at<float>(1, 0);
	float R31 = rotMat.at<float>(2, 0);

	float teta1 = -asin(R31);
	float teta2 = PI - teta1;
	float fi1 = 0;
	float fi2 = 0;

	if (fabs(R31) >= 1.001 || fabs(R31) < 0.999) {

		fi1 = atan2(R21 / cos(teta1), R11 / cos(teta1));
		fi2 = atan2(R21 / cos(teta2), R11 / cos(teta2));
	} else {
		fi1 = 0.0;
		fi2 = 0.0;
	}

	return fi1;
}

void LocalPlanner::updateHistogram(std::vector<float>& histSectors,
									cv::Mat posImuMapCenter,
									cv::Mat constraints) {

	float angPosition;
	float density;

	//TODO Change posImuMapCener to posRobotMapCenter
	//TODO Change units to mm instead of cells
	const float maxDistanceSq = 2*pow(MAP_SIZE * MAP_RASTER_SIZE, 2);
	const float hist_A = 1;
	const float hist_B = hist_A / maxDistanceSq;
	/// Get current Constraints Raster Map
	//Mat constraints = robot->getMovementConstraints();
	//Mat posImuMapCenter = robot->getPosImuConstraintsMapCenter();

	// update position of the robot
	float curRobX = posImuMapCenter.at<float>(0, 3);
	float curRobY = posImuMapCenter.at<float>(1, 3);

	//cout<<"RobotX"<<curRobCellX<<endl;
	//cout<<"RobotY"<<curRobCellY<<endl;


	/// for each cell of the raster map
	for (int x = 0; x < MAP_SIZE; x++) {
		for (int y = 0; y < MAP_SIZE; y++) {
			float cellX = ((x + 0.5)  - MAP_SIZE/2)*MAP_RASTER_SIZE;
			float cellY = ((y + 0.5)  - MAP_SIZE/2)*MAP_RASTER_SIZE;
			///angular position
//			angPosition = 180/PI*atan2(float(x - curRobCellX), y - curRobCellY);
			angPosition = atan2(float(cellY - curRobY), float(cellX - curRobX)) * 180 / PI;
			//cout<<"angPosition"<<angPosition<<endl;
			/// value of obstacle vector
			float c = pow(constraints.at<float>(x, y), 2);
			float d2 = pow((float) (cellX - curRobX), 2)
					+ pow((float) (cellY - curRobY), 2);
			density = c * (hist_A - hist_B * d2);

			//cout << "map (" << x << ", " << y << "), histB = " << hist_B << ", c = " << c << ", d2 = " << d2 << ", density = " << density << endl;
			/// update proper sector of histogram
			if (angPosition > 179.999)
				angPosition = -180.0;
			histSectors.at(floor(((angPosition + 180) / localPlannerParams.histResolution))) +=
					density;
		}
	}

//	for (int sect = 0; sect < numHistSectors; sect++) {
//		std::cout << "Angular histogram values for " << sect * localPlannerParams.histResolution - 180
//				<< " is " << histSectors.at(sect) << endl;
//	}
}

void LocalPlanner::smoothHistogram(std::vector<float>& histSectors) {

	int kernelSize = 2*localPlannerParams.gauss3sig/localPlannerParams.histResolution;
	kernelSize += (kernelSize + 1) % 2;	//kernelSize must be odd
	int kernelMid = kernelSize/2;
	cv::Mat gaussKernel = getGaussianKernel(kernelSize,
											(localPlannerParams.gauss3sig/localPlannerParams.histResolution)/3,
											CV_32FC1);
	gaussKernel *= 1/gaussKernel.at<float>(kernelMid);
	//cout << "gaussKernel = " << gaussKernel << endl;
	vector<float> newHistSectors(histSectors.size(), 0);
	for(int i = 0; i < histSectors.size(); i++){
		for(int k = 0; k < kernelSize; k++){
			newHistSectors[i] += histSectors[(i + k - kernelMid + histSectors.size()) % histSectors.size()]
			                      * gaussKernel.at<float>(k);
		}
	}
	histSectors = newHistSectors;
}

void LocalPlanner::findFreeSectors(std::vector<float>& histSectors,
									std::vector<int>& freeSectors) {

	freeSectors.clear();

	bool needToHigherThreshold = true;

	for (int thresholdIndex = 0; thresholdIndex < 5; thresholdIndex++) {
		for (int sec = 0; sec < numHistSectors; sec++) {

			if (fabs(histSectors.at(sec)) <= localPlannerParams.threshold + thresholdIndex) {
				freeSectors.push_back(sec);
				needToHigherThreshold = false;
			}
		}
		if (!needToHigherThreshold) {
			break;
		}
	}
	cout << "Wolne sektory:" << endl;
	for (int i = 0; i < freeSectors.size(); i++) {
		cout << "Free to go: " << freeSectors.at(i) * localPlannerParams.histResolution << " "
				<< (freeSectors.at(i) + 1) * localPlannerParams.histResolution << endl;
	}
}

/// return Goal Direction converted from Global Map to
/// local MAP
float LocalPlanner::determineGoalInLocalMap(cv::Mat posLocalToGlobalMap,
											float goalDirGlobalMap) {

	float globalYaw = 0.0;
	float localToGlobalDifference = 0.0;

	//cout<<"determineGoalInLocalMap"<<endl;

	/// get Orientation of Local Map in Global Map
	//Mat posLocalToGlobalMap = robot->getLocalMapPosInGlobalMap();
	/// and convert this orentation to euler yaw
	globalYaw = RotMatToEulerYaw(posLocalToGlobalMap);
	globalYaw = globalYaw * 180 / PI;
	cout << "Global_YAW: " << globalYaw << endl;
	// calculate difference between local and global map in orientation
	localToGlobalDifference = goalDirGlobalMap - globalYaw;
	cout << "Difference: " << localToGlobalDifference << endl;

	//cout<<"determineGoalInLocalMap"<<endl;
	return goalDirGlobalMap - globalYaw;
}

float LocalPlanner::findOptimSector(const std::vector<int>& freeSectors,
									float goalDirLocalMap) {

	cout << "calculateLocalDirection START" << endl;
	// goal direction+offset/ sectors alpha
	int goalSector = (goalDirLocalMap + 180) / localPlannerParams.histResolution;
	if (goalSector == localPlannerParams.histResolution) {
		goalSector = 0;
	}
	cout << "Goal sector : " << goalSector << endl;
	int foundSector = -1;
	int selectedSector = 0;

	if (freeSectors.size() == 0) {
		std::cout << "No free sectors -- I'm stuck !!!!" << std::endl;
	}
	/// if goal sector is free sector
	cout << "Free sector size : " << freeSectors.size() << endl;
	int bestSectorID = -1;
	int bestSectorDistance = -1;
	for (int i = 0; i < freeSectors.size(); i++) {
		cout << "Free vs goal : " << freeSectors[i] << " " << goalSector
				<< endl;
		int distance = (freeSectors[i] - goalSector)
				* (freeSectors[i] - goalSector);
		cout << "Distance " << distance << endl;
		if (bestSectorID == -1 || bestSectorDistance > distance) {
			bestSectorDistance = distance;
			bestSectorID = freeSectors[i];
		}
	}

	//middle of best sector
	float bestDirection = bestSectorID * localPlannerParams.histResolution + localPlannerParams.histResolution/2 - 180;
	cout << "FoundSector: " << bestSectorID << endl;
	cout << "BestSector distance: " << bestSectorDistance << endl;
	cout << "calculateLocalDirection END" << endl;
	return bestDirection;
}

void LocalPlanner::determineDriversCommand(cv::Mat posImuMapCenter,
											float bestDirLocalMap) {

	//Mat posImuMapCenter = robot->getPosImuConstraintsMapCenter();
	float localYaw = RotMatToEulerYaw(posImuMapCenter);
	localYaw = localYaw * 180 / PI;
	cout << "localYaw: " << localYaw << endl;
	cout << "Best Direction: " << bestDirLocalMap << endl;

	// if we achieve set point direction then go straight ahead
	// 100 means that we are in target direction with 10 degree margin
	if ((bestDirLocalMap - localYaw)
			* (bestDirLocalMap - localYaw)< localPlannerParams.steeringMargin * localPlannerParams.steeringMargin)
	{
		cout<<"Straight"<<endl;

		globalPlanner->setMotorsVel(25, 25);
	}
	else if (bestDirLocalMap > localYaw)	{
		cout<<"Right"<<endl;

		globalPlanner->setMotorsVel(40, -40);
	}
	else{
		cout<<"Left"<<endl;

		globalPlanner->setMotorsVel(-40, 40);
	}
	cout << "End LocalPlanner::determineDriversCommand" << endl;
	//getchar();
}

void LocalPlanner::stopThread() {
	localPlannerParams.runThread = false;
	if (localPlannerThread.joinable()) {
		localPlannerThread.join();
	}
}

void LocalPlanner::getVecFieldHist(std::vector<float>& retVecFieldHist,
									float& retGoalDir,
									float& retBestDirection){
	std::unique_lock<std::mutex> lck(mtxVecFieldHist);
	retVecFieldHist = vecFieldHist;
	retGoalDir = shGoalDirLocalMap;
	retBestDirection = shBestDirLocalMap;
	lck.unlock();
}

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

	if ( localPlannerParams.avoidObstacles == 1)
	{
		robot->getLocalPlanData(constraints, posImuMapCenter,
				posLocalToGlobalMap);

		if(constraints.empty() || posImuMapCenter.empty() || posLocalToGlobalMap.empty()){
			cout << "No movement constraints data" << endl;
			return;
		}

		initHistogram();
		//cout << "updateHistogram()" << endl;
		updateHistogram();
		//cout << "findFreeSectors()" << endl;
		findFreeSectors();
		//cout << "smoothHistogram" << endl;
		smoothHistogram();

		//cout << "Waiting for mtxVecFieldHist" << endl;
		std::unique_lock<std::mutex> lck(mtxVecFieldHist);
		vecFieldHist = histSectors;
		//cout << "vecFieldHist.size() = " << vecFieldHist.size() << endl;
		lck.unlock();
	}


	setGoalDirection();
	/// goal direction in the local map coordiantes
	determineGoalInLocalMap();

	std::unique_lock<std::mutex> lck(mtxVecFieldHist);
	/// optimal direction in the local map - nearest to the goal
	if ( localPlannerParams.avoidObstacles == 1)
	{
		findOptimSector();
	}
	else
	{
		bestDirection = goalDirection;
	}
	lck.unlock();

	determineDriversCommand();

}

float LocalPlanner::getLocalDirection() {
	return bestDirection;
}

void LocalPlanner::setGoalDirection() {

	///// this part of code is only for  test purposes
	// to test the straight ahead direction of movement:
	// logic flow: get current orientation ane keep it
	static bool setStraighAheadDirection = false;
	static float direction = 0;

	if (!setStraighAheadDirection) {
		//Mat posLocalToGlobalMap = robot->getLocalMapPosInGlobalMap();
		/// and convert this orentation to euler yaw
		direction = RotMatToEulerYaw(posLocalToGlobalMap);
		direction = 180 / PI * direction;
		setStraighAheadDirection = true;
	}
	///////////////

	goalDirection = globalPlanner->getHeadingToGoal();
//	goalDirection = direction; // => test go straight ahead
}

void LocalPlanner::initHistogram() {

	histSectors = std::vector<float>(HIST_SECTORS, 0.0);
}

void LocalPlanner::localPlanerTest() {

	initHistogram();
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

void LocalPlanner::updateHistogram() {

	float angPosition;
	float density;

	const float maxDistance = 1.41 * (float) MAP_SIZE * 1.41 * (float) MAP_SIZE;
	const float hist_A = 1;
	const float hist_B = hist_A / maxDistance;
	/// Get current Constraints Raster Map
	//Mat constraints = robot->getMovementConstraints();
	//Mat posImuMapCenter = robot->getPosImuConstraintsMapCenter();

	// update position of the robot
	float addX = posImuMapCenter.at<float>(0, 3) / MAP_RASTER_SIZE;
	float addY = posImuMapCenter.at<float>(1, 3) / MAP_RASTER_SIZE;

	int curRobCellX = MAP_SIZE / 2 + addX;
	int curRobCellY = MAP_SIZE / 2 + addY;
	int sector = 0;
	//cout<<"RobotX"<<curRobCellX<<endl;
	//cout<<"RobotY"<<curRobCellY<<endl;


	/// for each cell of the raster map
	for (int x = 0; x < MAP_SIZE; x++) {
		for (int y = 0; y < MAP_SIZE; y++) {
			///angular position
//			angPosition = 180/PI*atan2(float(x - curRobCellX), y - curRobCellY);
			angPosition = 180 / PI
					* atan2(float(y - curRobCellY), float(x - curRobCellX));
			//cout<<"angPosition"<<angPosition<<endl;
			/// value of obstacle vector
			float c = pow(constraints.at<float>(x, y), 2);
			float d2 = pow((float) (x - curRobCellX), 2)
					+ pow((float) (y - curRobCellY), 2);
			density = c * (hist_A - hist_B * d2);

			//cout << "map (" << x << ", " << y << "), histB = " << hist_B << ", c = " << c << ", d2 = " << d2 << ", density = " << density << endl;
			/// update proper sector of histogram
			if (angPosition > 179.999)
				angPosition = -180.0;
			histSectors.at(floor(((angPosition + 180) / HIST_ALPHA))) +=
					density;
		}
	}

	for (int sect = 0; sect < HIST_SECTORS; sect++) {
		std::cout << "Angular histogram values for " << sect * HIST_ALPHA - 180
				<< " is " << histSectors.at(sect) << endl;
	}
}

void LocalPlanner::smoothHistogram() {

	cv::Mat src = cv::Mat::ones(10, 1, CV_32F), dst = cv::Mat(10, 1, CV_32F);

	// 3 sigmas is 25 degrees
	cv::GaussianBlur(src, dst, cv::Size(10, 1), (25.0 / HIST_ALPHA) / 3, 0);

	for (int i = 0; i < 10; i++) {
		printf("Tab value : %f\n", dst.at<float>(i));
	}

	//int a;
	//cin >> a;
}

void LocalPlanner::findFreeSectors() {

	freeSectors.clear();

	bool needToHigherThreshold = true;

	for (int thresholdIndex = 0; thresholdIndex < 5; thresholdIndex++) {
		for (int sec = 0; sec < HIST_SECTORS; sec++) {

			if (fabs(histSectors.at(sec)) <= HIST_THRESHOLD + thresholdIndex) {
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
		cout << "Free to go: " << freeSectors.at(i) * HIST_ALPHA << " "
				<< (freeSectors.at(i) + 1) * HIST_ALPHA << endl;
	}
}

/// return Goal Direction converted from Global Map to
/// local MAP
void LocalPlanner::determineGoalInLocalMap() {

	float globalYaw = 0.0;
	float localToGlobalDifference = 0.0;

	//cout<<"determineGoalInLocalMap"<<endl;

	/// get Orientation of Local Map in Global Map
	//Mat posLocalToGlobalMap = robot->getLocalMapPosInGlobalMap();
	/// and convert this orentation to euler yaw
	globalYaw = RotMatToEulerYaw(posLocalToGlobalMap);
	globalYaw = 180 / PI * globalYaw;
	cout << "Global_YAW: " << globalYaw << endl;
	// calculate difference between local and global map in orientation
	localToGlobalDifference = goalDirection - globalYaw;
	cout << "Difference: " << localToGlobalDifference << endl;
	goalDirection = goalDirection - globalYaw;

	//cout<<"determineGoalInLocalMap"<<endl;
}

void LocalPlanner::findOptimSector() {

	cout << "calculateLocalDirection START" << endl;
	// goal direction+offset/ sectors alpha
	int goalSector = (goalDirection + 180) / HIST_ALPHA;
	if (goalSector == HIST_ALPHA) {
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

	bestDirection = bestSectorID * HIST_ALPHA;
	cout << "FoundSector: " << bestSectorID << endl;
	cout << "BestSector distance: " << bestSectorDistance << endl;
	cout << "calculateLocalDirection END" << endl;
}

void LocalPlanner::determineDriversCommand() {

	//Mat posImuMapCenter = robot->getPosImuConstraintsMapCenter();
	float localYaw = RotMatToEulerYaw(posImuMapCenter);
	localYaw = localYaw * 180 / PI;
	cout << "localYaw: " << localYaw << endl;
	bestDirection = bestDirection - 180.0;
	cout << "Best Direction: " << bestDirection << endl;

	// if we achieve set point direction then go straight ahead
	// 100 means that we are in target direction with 10 degree margin
	if ((bestDirection - localYaw)
			* (bestDirection - localYaw)< STEERING_MARGIN * STEERING_MARGIN)
	{
		cout<<"Straight"<<endl;

		//globalPlanner->setMotorsVel(25, 25);
	}
	else if (bestDirection > localYaw)	{
		cout<<"Right"<<endl;

		//globalPlanner->setMotorsVel(25, -25);
	}
	else{
		cout<<"Left"<<endl;

		//globalPlanner->setMotorsVel(-25, 25);
	}
	//getchar();
}

void LocalPlanner::stopThread() {
	runThread = false;
	if (localPlannerThread.joinable()) {
		localPlannerThread.join();
	}
}

void LocalPlanner::getVecFieldHist(std::vector<float>& retVecFieldHist, float& retBestDirection){
	std::unique_lock<std::mutex> lck(mtxVecFieldHist);
	retVecFieldHist = vecFieldHist;
	retBestDirection = bestDirection;
	lck.unlock();
}

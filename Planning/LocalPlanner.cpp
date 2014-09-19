/*
 * LocalPlanner.cpp
 *
 *      Author: sebastian
 */

#include <chrono>
#include "LocalPlanner.h"

using namespace cv;
using namespace std;

#define LEFT_CHANNEL 2
#define RIGHT_CHANNEL 1

LocalPlanner::LocalPlanner(Robot* irobot, GlobalPlanner* planer,
		TiXmlElement* settings) :
		robot(irobot),
		globalPlanner(planer),
		startOperate(false),
		curSpeedMax(20),
		prevCurSpeed(20)
{
	cout << "LocalPlanner()" << endl;

	readSettings(settings);
	numHistSectors = (float)360/localPlannerParams.histResolution + 0.5;

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
	if (pLocalPlanner->QueryIntAttribute("debug", &localPlannerParams.debug)
			!= TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for localPlanner debug";
	}
	if (pLocalPlanner->QueryIntAttribute("avoidObstacles", &localPlannerParams.avoidObstacles)
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
	if (pLocalPlanner->QueryFloatAttribute("VFH_MaxDistance",
			&localPlannerParams.maxDistance) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for VFH_MaxDistance";
	}
	if (pLocalPlanner->QueryFloatAttribute("VFH_BackwardsPenalty",
			&localPlannerParams.backwardsPenalty) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for VFH_BackwardsPenalty";
	}
	if (pLocalPlanner->QueryFloatAttribute("VFH_NormalSpeed",
			&localPlannerParams.normalSpeed) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for VFH_NormalSpeed";
	}
	if (pLocalPlanner->QueryFloatAttribute("VFH_PreciseSpeed",
			&localPlannerParams.preciseSpeed) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for VFH_PreciseSpeed";
	}
	if (pLocalPlanner->QueryFloatAttribute("VFH_GentleTurnMargin",
			&localPlannerParams.gentleTurnMargin) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for VFH_GentleTurnMargin";
	}
	if (pLocalPlanner->QueryFloatAttribute("VFH_GentleTurnSpeedDiff",
			&localPlannerParams.gentleTurnSpeedDiff) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for VFH_GentleTurnSpeedDiff";
	}
	if (pLocalPlanner->QueryFloatAttribute("VFH_TurnSpeed",
			&localPlannerParams.turnSpeed) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for VFH_TurnSpeed";
	}
	if (pLocalPlanner->QueryIntAttribute("VFH_TurnTimeout",
			&localPlannerParams.turnTimeout) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for VFH_TurnTimeout";
	}
	if (pLocalPlanner->QueryIntAttribute("VFH_InterruptTime",
			&localPlannerParams.interruptTime) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for VFH_InterruptTime";
	}
	if (pLocalPlanner->QueryFloatAttribute("VFH_InterruptSpeed",
			&localPlannerParams.interruptSpeed) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for VFH_InterruptSpeed";
	}
	if (pLocalPlanner->QueryFloatAttribute("VFH_ImuAccVarianceLimit",
			&localPlannerParams.imuAccVarianceLimit) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for VFH_ImuAccVarianceLimit";
	}

	printf("LocalPlanner -- runThread: %d\n", localPlannerParams.runThread);
	printf("LocalPlanner -- debug: %d\n", localPlannerParams.debug);
	printf("LocalPlanner -- avoidObstacles: %d\n", localPlannerParams.avoidObstacles);
	printf("LocalPlanner -- VFH_HistResolution: %d\n",
			localPlannerParams.histResolution);
	printf("LocalPlanner -- VFH_Threshold: %f\n", localPlannerParams.threshold);
	printf("LocalPlanner -- VFH_SteeringMargin: %f\n",
				localPlannerParams.steeringMargin);
	printf("LocalPlanner -- VFH_Gauss3sig: %f\n",
			localPlannerParams.gauss3sig);
	printf("LocalPlanner -- VFH_MaxDistance: %f\n",
			localPlannerParams.maxDistance);
	printf("LocalPlanner -- VFH_NormalSpeed: %f\n",
				localPlannerParams.normalSpeed);
	printf("LocalPlanner -- VFH_PreciseSpeed: %f\n",
				localPlannerParams.preciseSpeed);
	printf("LocalPlanner -- VFH_TurnSpeed: %f\n",
				localPlannerParams.turnSpeed);
	printf("LocalPlanner -- VFH_TurnTimeout: %d\n",
				localPlannerParams.turnTimeout);
	printf("LocalPlanner -- VFH_InterruptTime: %d\n",
				localPlannerParams.interruptTime);
	printf("LocalPlanner -- VFH_InterruptSpeed: %f\n",
				localPlannerParams.interruptSpeed);
	printf("LocalPlanner -- VFH_ImuAccVarianceLimit: %f\n",
				localPlannerParams.imuAccVarianceLimit);

}

void LocalPlanner::startLocalPlanner() {
	cout<<"LocalPlanner::startLocalPlanner()"<<endl;
	startOperate = true;
}

void LocalPlanner::stopLocalPlanner() {
	cout << "LocalPlanner::stopLocalPlanner()" << endl;
	startOperate = false;
}

void LocalPlanner::setPreciseSpeed() {
	std::unique_lock<std::mutex> lck(mtxCurSpeed);
	prevCurSpeed = curSpeedMax = localPlannerParams.preciseSpeed;
	lck.unlock();
}

void LocalPlanner::setNormalSpeed() {
	std::unique_lock<std::mutex> lck(mtxCurSpeed);
	prevCurSpeed = curSpeedMax = localPlannerParams.normalSpeed;
	lck.unlock();
}


void LocalPlanner::run() {
	try{
		while (localPlannerParams.runThread) {
			if (startOperate) {
				executeVFH();
			}
			else{
				globalPlanner->setMotorsVel(0, 0);
			}
			std::chrono::milliseconds duration(50);
			std::this_thread::sleep_for(duration);
		}
		globalPlanner->setMotorsVel(0, 0);
	}
	catch(char const* error){
		cout << error << endl;
	}
	catch(...){
		cout << "LocalPlanner unrecognized exception" << endl;
		exit(-1);
	}
}

void LocalPlanner::executeVFH() {


	//constraint data
	Mat constraints;
	Mat posRobotMapCenter;
	Mat posLocalToGlobalMap;

	robot->getLocalPlanData(constraints, posRobotMapCenter,
			posLocalToGlobalMap);
	if(!constraints.empty() && !posRobotMapCenter.empty() && !posLocalToGlobalMap.empty()){
		vector<float> histSectors(numHistSectors, 0);
		//vector<int> freeSectors;

		//cout << "updateHistogram()" << endl;
		updateHistogram(histSectors, posRobotMapCenter, constraints);
		//cout << "smoothHistogram" << endl;
		smoothHistogram(histSectors);
		//cout << "findFreeSectors()" << endl;
		//findFreeSectors(histSectors, freeSectors);


		float goalDirGlobalMap = setGoalDirection(posLocalToGlobalMap);
		/// goal direction in the local map coordiantes
		float goalDirLocalMap = determineGoalInLocalMap(posLocalToGlobalMap, goalDirGlobalMap);

		float bestDirLocalMap = 0;
		/// optimal direction in the local map - nearest to the goal
		bestDirLocalMap = findOptimSector(histSectors,
										posRobotMapCenter,
										goalDirLocalMap);

		determineDriversCommand(posRobotMapCenter,
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
	static const bool forwardRun = false;
	static bool setStraighAheadDirection = false;
	static float direction = 0;

	if(forwardRun){
		if (!setStraighAheadDirection) {
			float goalDirection = globalPlanner->getHeadingToGoal();
			//Mat posLocalToGlobalMap = robot->getLocalMapPosInGlobalMap();
			/// and convert this orentation to euler yaw
			direction = rotMatToEulerYaw(posLocalToGlobalMap);
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

float LocalPlanner::rotMatToEulerYaw(Mat rotMat) {

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
									cv::Mat posRobotMapCenter,
									cv::Mat constraints) {

	float angPosition;
	float density;

	const float hist_A = 1;
	const float hist_B = hist_A / localPlannerParams.maxDistance;
	/// Get current Constraints Raster Map
	//Mat constraints = robot->getMovementConstraints();
	//Mat posRobotMapCenter = robot->getPosImuConstraintsMapCenter();

	// update position of the robot
	float curRobX = posRobotMapCenter.at<float>(0, 3);
	float curRobY = posRobotMapCenter.at<float>(1, 3);

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
			float d = sqrt(pow((float) (cellX - curRobX), 2)+ pow((float) (cellY - curRobY), 2));
			density = c * max(hist_A - hist_B * d, 0.0f);

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

/*void LocalPlanner::findFreeSectors(std::vector<float>& histSectors,
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
//	cout << "Wolne sektory:" << endl;
//	for (int i = 0; i < freeSectors.size(); i++) {
//		cout << "Free to go: " << freeSectors.at(i) * localPlannerParams.histResolution << " "
//				<< (freeSectors.at(i) + 1) * localPlannerParams.histResolution << endl;
//	}
}*/

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
	globalYaw = rotMatToEulerYaw(posLocalToGlobalMap);
	globalYaw = globalYaw * 180 / PI;
//	cout << "Global_YAW: " << globalYaw << endl;
	// calculate difference between local and global map in orientation
	localToGlobalDifference = goalDirGlobalMap - globalYaw;
//	cout << "Difference: " << localToGlobalDifference << endl;

	//cout<<"determineGoalInLocalMap"<<endl;
	return goalDirGlobalMap - globalYaw;
}

float LocalPlanner::findOptimSector(const std::vector<float>& histSectors,
									cv::Mat posRobotMapCenter,
									float goalDirLocalMap) {


//	cout << "calculateLocalDirection START" << endl;
	// goal direction+offset/ sectors alpha

	float robotDirLocalMap = rotMatToEulerYaw(posRobotMapCenter);
	robotDirLocalMap = robotDirLocalMap * 180 / PI;

	/// if goal sector is free sector
//	cout << "Free sector size : " << freeSectors.size() << endl;
	int bestSectorID = -1;
	float bestSectorScore = -1;
	for (int i = 0; i < histSectors.size(); i++) {
		//cout << "Free vs goal : " << freeSectors[i] << " " << goalSector
		//		<< endl;
		float sectorDirLocalMap = (i + 0.5) * localPlannerParams.histResolution - 180;
		float sectorToRobotDiff = min(fabs(sectorDirLocalMap - robotDirLocalMap), fabs(fabs(sectorDirLocalMap - robotDirLocalMap) - 360));

		float score = min(fabs(goalDirLocalMap - sectorDirLocalMap), fabs(fabs(goalDirLocalMap - sectorDirLocalMap) - 360))/180;	//angular distance to goal penalty
		score += (sectorToRobotDiff > 90) ? localPlannerParams.backwardsPenalty : 0.0f;
		score += histSectors[i];

		if(localPlannerParams.debug >= 2){
			cout << "sector " << i << ", score angular = ";
			cout <<	min(fabs(goalDirLocalMap - sectorDirLocalMap), fabs(fabs(goalDirLocalMap - sectorDirLocalMap) - 360))/180;
			cout << ", score backwards = " << ((sectorToRobotDiff > 90) ? localPlannerParams.backwardsPenalty : 0.0f);
			cout << ", score histogram = " << histSectors[i] << endl;
			cout << "score = " << score << endl;
		}

		if (bestSectorID == -1 || bestSectorScore > score) {
			bestSectorScore = score;
			bestSectorID = i;
		}
	}

	//middle of best sector
	float bestDirection = (bestSectorID + 0.5) * localPlannerParams.histResolution - 180;
	if(localPlannerParams.debug >= 1){
		cout << "FoundSector: " << bestSectorID << endl;
		cout << "BestSector score: " << bestSectorScore << endl;
	}
	return bestDirection;
}

void LocalPlanner::determineDriversCommand(cv::Mat posRobotMapCenter,
											float bestDirLocalMap)
{
	static std::chrono::high_resolution_clock::time_point startTurnTime = std::chrono::high_resolution_clock::now();
	static bool turningRightStarted = false;
	static bool turningLeftStarted = false;
	static std::chrono::high_resolution_clock::time_point startInterruptTime = std::chrono::high_resolution_clock::now();
	static bool turnInterrupted = false;
	//Mat posRobotMapCenter = robot->getPosImuConstraintsMapCenter();
	float localYaw = rotMatToEulerYaw(posRobotMapCenter);
	localYaw = localYaw * 180 / PI;
	if (localPlannerParams.debug == 1)
	{
		cout << "localYaw: " << localYaw << endl;
		cout << "Best Direction: " << bestDirLocalMap << endl;
	}
	if(localPlannerParams.avoidObstacles == 1){
		// if we achieve set point direction then go straight ahead
		// 100 means that we are in target direction with 10 degree margin
		if (localPlannerParams.debug >= 2)
		{
			cout << "turningRightStarted = " << turningRightStarted << endl;
			cout << "turningLeftStarted = " << turningLeftStarted << endl;
		}
		if((turningLeftStarted || turningRightStarted) &&
			std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTurnTime).count() > localPlannerParams.turnTimeout)
		{
			if (localPlannerParams.debug >= 1)
			{
				cout << "Deadlock detected -> interrupting turning" << endl;
			}
			turnInterrupted = true;
			startInterruptTime = std::chrono::high_resolution_clock::now();
			turningRightStarted = false;
			turningLeftStarted = false;
		}

		if(turnInterrupted &&
			std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startInterruptTime).count() < localPlannerParams.interruptTime)
		{
			if (localPlannerParams.debug >= 1){
				cout<<"Interrupted"<<endl;
			}

			float curSpeed = determineCurSpeed();

			globalPlanner->setMotorsVel(-localPlannerParams.interruptSpeed, -localPlannerParams.interruptSpeed);
		}
		else if (fabs(bestDirLocalMap - localYaw) < localPlannerParams.steeringMargin)
		{
			if (localPlannerParams.debug >= 1){
				cout<<"Straight"<<endl;
			}

			float curSpeed = determineCurSpeed();

			globalPlanner->setMotorsVel(curSpeed, curSpeed);
			turningRightStarted = false;
			turningLeftStarted = false;
			turnInterrupted = false;
		}
		else if ((bestDirLocalMap - localYaw > 0) && (bestDirLocalMap - localYaw < localPlannerParams.gentleTurnMargin))	{
			if (localPlannerParams.debug >= 1){
				cout<<"Gently right"<<endl;
			}

			globalPlanner->setMotorsVel(localPlannerParams.turnSpeed, localPlannerParams.turnSpeed - localPlannerParams.gentleTurnSpeedDiff);
			if(!turningRightStarted){
				startTurnTime = std::chrono::high_resolution_clock::now();
			}
			turningRightStarted = true;
			turningLeftStarted = false;
			turnInterrupted = false;
		}
		else if (bestDirLocalMap - localYaw > 0)	{
			if (localPlannerParams.debug >= 1){
				cout<<"Right"<<endl;
			}

			globalPlanner->setMotorsVel(localPlannerParams.turnSpeed, -localPlannerParams.turnSpeed);
			if(!turningRightStarted){
				startTurnTime = std::chrono::high_resolution_clock::now();
			}
			turningRightStarted = true;
			turningLeftStarted = false;
			turnInterrupted = false;
		}
		else if ((bestDirLocalMap - localYaw < 0) && (bestDirLocalMap - localYaw > -localPlannerParams.gentleTurnMargin))	{
			if (localPlannerParams.debug >= 1){
				cout<<"Gently left"<<endl;
			}

			globalPlanner->setMotorsVel(localPlannerParams.turnSpeed - localPlannerParams.gentleTurnSpeedDiff, localPlannerParams.turnSpeed);
			if(!turningLeftStarted){
				startTurnTime = std::chrono::high_resolution_clock::now();
			}
			turningRightStarted = false;
			turningLeftStarted = true;
			turnInterrupted = false;
		}
		else{
			if (localPlannerParams.debug >= 1){
				cout<<"Left"<<endl;
			}

			globalPlanner->setMotorsVel(-localPlannerParams.turnSpeed, localPlannerParams.turnSpeed);
			if(!turningLeftStarted){
				startTurnTime = std::chrono::high_resolution_clock::now();
			}
			turningRightStarted = false;
			turningLeftStarted = true;
			turnInterrupted = false;
		}
	}
	else{
		if (fabs(bestDirLocalMap - localYaw) < localPlannerParams.gentleTurnMargin)
		{
			if (localPlannerParams.debug >= 1){
				cout<<"Straight"<<endl;
			}

			float curSpeed = determineCurSpeed();

			globalPlanner->setMotorsVel(curSpeed, curSpeed);
		}
		else{
			if (localPlannerParams.debug >= 1){
				cout<<"Stop"<<endl;
			}
			globalPlanner->setMotorsVel(0, 0);
		}
	}
//	cout << "End LocalPlanner::determineDriversCommand" << endl;
	//getchar();
}

float LocalPlanner::determineCurSpeed(){
	float curImuAccVariance = robot->getImuAccVariance();
	std::unique_lock<std::mutex> lck(mtxCurSpeed);
	static const float accVarianceLimit = 0.1;
	float curSpeed = prevCurSpeed;
	if(curImuAccVariance < localPlannerParams.imuAccVarianceLimit){
		curSpeed = min(curSpeed + 0.5f, curSpeedMax);
		if (localPlannerParams.debug >= 1){
			cout<<"Speeding up"<<endl;
		}
	}
	if(curImuAccVariance > localPlannerParams.imuAccVarianceLimit){
		curSpeed = max(curSpeed - 0.5f, 0.0f);
		if (localPlannerParams.debug >= 1){
			cout<<"Slowing down"<<endl;
		}
	}
	prevCurSpeed = curSpeed;
	lck.unlock();
	return curSpeed;
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

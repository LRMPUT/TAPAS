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
	planning_sub = nh.subscribe("planning_data", 10, &LocalPlanner::planningCallback, this);

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

	/*if (pLocalPlanner->QueryFloatAttribute("VFH_Threshold",
			&localPlannerParams.threshold) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for localPlanner VFH_Threshold";
	}*/
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
	//printf("LocalPlanner -- VFH_Threshold: %f\n", localPlannerParams.threshold);
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

void LocalPlanner::planningCallback(const TAPAS::PlanningData& msg) {
	constraints = RosHelpers::readMatrixMsg(msg.constraintsMap);
	posRobotMapCenter = RosHelpers::readMatrixMsg(msg.posRobotMapCenter);
	posLocalToGlobalMap = RosHelpers::readMatrixMsg(msg.globalMapCenter);
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
//			cout << "localPlanner thread" << endl;
			if (startOperate) {
//				cout << "executeVFH" << endl;
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
		cout << "Char exception in LocalPlanner: " << error << endl;
		exit(1);
	}
	catch(std::exception& e){
		cout << "Std exception in LocalPlanner: " << e.what() << endl;
		exit(1);
	}
	catch(...){
		cout << "Unexpected exception in LocalPlanner" << endl;
		exit(1);
	}
}

void LocalPlanner::executeVFH() {


	//constraint data
	Mat constraints;
	Mat posRobotMapCenter;
	Mat posLocalToGlobalMap;

	if(!constraints.empty() && !posRobotMapCenter.empty() && !posLocalToGlobalMap.empty()){
		vector<float> histSectors(numHistSectors, 0);
		//vector<int> freeSectors;

		//cout << "updateHistogram()" << endl;
		PlannerHelpers::updateHistogram(histSectors,
						posRobotMapCenter,
						constraints,
						localPlannerParams);
		//cout << "smoothHistogram" << endl;
		PlannerHelpers::smoothHistogram(histSectors, localPlannerParams);
		//cout << "findFreeSectors()" << endl;
		//findFreeSectors(histSectors, freeSectors);


		float goalDirGlobalMap = setGoalDirection(posLocalToGlobalMap);
		/// goal direction in the local map coordiantes
		float goalDirLocalMap = PlannerHelpers::determineGoalInLocalMap(posLocalToGlobalMap, goalDirGlobalMap);

		float bestDirLocalMap = 0;
		/// optimal direction in the local map - nearest to the goal
		bestDirLocalMap = PlannerHelpers::findOptimSector(histSectors,
										posRobotMapCenter,
										goalDirLocalMap,
										localPlannerParams);

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
			direction = PlannerHelpers::rotMatToEulerYaw(posLocalToGlobalMap);
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

void LocalPlanner::determineDriversCommand(cv::Mat posRobotMapCenter,
											float bestDirLocalMap)
{
	static queue<std::chrono::high_resolution_clock::time_point> interruptsQueue;
	static const std::chrono::milliseconds interruptQueueTime(60000);
	static bool powerInterruptStarted = false;
	if(interruptsQueue.size() > 0){
		while(interruptsQueue.front() < std::chrono::high_resolution_clock::now() - interruptQueueTime){
			interruptsQueue.pop();
			if(interruptsQueue.size() == 0){
				break;
			}
		}
	}
	static std::chrono::high_resolution_clock::time_point startTurnTime = std::chrono::high_resolution_clock::now();
	static bool turningStarted = false;
	//static bool turningLeftStarted = false;
	static std::chrono::high_resolution_clock::time_point startInterruptTime = std::chrono::high_resolution_clock::now();
	static bool turnInterrupted = false;
	//Mat posRobotMapCenter = robot->getPosImuConstraintsMapCenter();
	float localYaw = PlannerHelpers::rotMatToEulerYaw(posRobotMapCenter);
	localYaw = localYaw * 180 / PI;
	if (localPlannerParams.debug >= 1)
	{
		cout << "localYaw: " << localYaw << endl;
		cout << "Best Direction: " << bestDirLocalMap << endl;
		cout << "interruptsQueue.size() = " << interruptsQueue.size() << endl;
	}
	if(localPlannerParams.avoidObstacles == 1){
		// if we achieve set point direction then go straight ahead
		// 100 means that we are in target direction with 10 degree margin
		if (localPlannerParams.debug >= 2)
		{
			cout << "turningStarted = " << turningStarted << endl;
//			cout << "turningLeftStarted = " << turningLeftStarted << endl;
		}
		if((turningStarted) &&
			std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTurnTime).count() > localPlannerParams.turnTimeout)
		{
			if (localPlannerParams.debug >= 1)
			{
				cout << "Deadlock detected -> interrupting turning" << endl;
			}
			interruptsQueue.push(std::chrono::high_resolution_clock::now());
			turnInterrupted = true;
			startInterruptTime = std::chrono::high_resolution_clock::now();
			turningStarted = false;
		}

		if(turnInterrupted &&
			std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startInterruptTime).count() < localPlannerParams.interruptTime)
		{
			if (localPlannerParams.debug >= 1){
				cout<<"Interrupted"<<endl;
			}

			float curSpeed = determineCurSpeed();

			if(interruptsQueue.size() > 6){
				globalPlanner->setMotorsVel(-100, -100);
				powerInterruptStarted = true;
			}
			else{
				globalPlanner->setMotorsVel(-localPlannerParams.interruptSpeed, -localPlannerParams.interruptSpeed);
			}
		}
		else if (fabs(bestDirLocalMap - localYaw) < localPlannerParams.steeringMargin)
		{
			if (localPlannerParams.debug >= 1){
				cout<<"Straight"<<endl;
			}

			float curSpeed = determineCurSpeed();

			if(powerInterruptStarted == true){
				while(interruptsQueue.size() > 0){
					interruptsQueue.pop();
				}
				powerInterruptStarted = false;
			}
			globalPlanner->setMotorsVel(curSpeed, curSpeed);
			turningStarted = false;
			turnInterrupted = false;
		}
		else if ((bestDirLocalMap - localYaw > 0) && (bestDirLocalMap - localYaw < localPlannerParams.gentleTurnMargin))	{
			if (localPlannerParams.debug >= 1){
				cout<<"Gently right"<<endl;
			}

			if(powerInterruptStarted == true){
				while(interruptsQueue.size() > 0){
					interruptsQueue.pop();
				}
				powerInterruptStarted = false;
			}

			globalPlanner->setMotorsVel(localPlannerParams.turnSpeed, localPlannerParams.turnSpeed - localPlannerParams.gentleTurnSpeedDiff);
			if(!turningStarted){
				startTurnTime = std::chrono::high_resolution_clock::now();
			}
			turningStarted = true;
			turnInterrupted = false;
		}
		else if (bestDirLocalMap - localYaw > 0)	{
			if (localPlannerParams.debug >= 1){
				cout<<"Right"<<endl;
			}

			if(powerInterruptStarted == true){
				while(interruptsQueue.size() > 0){
					interruptsQueue.pop();
				}
				powerInterruptStarted = false;
			}

			globalPlanner->setMotorsVel(localPlannerParams.turnSpeed, -localPlannerParams.turnSpeed);
			if(!turningStarted){
				startTurnTime = std::chrono::high_resolution_clock::now();
			}
			turningStarted = true;
			turnInterrupted = false;
		}
		else if ((bestDirLocalMap - localYaw < 0) && (bestDirLocalMap - localYaw > -localPlannerParams.gentleTurnMargin))	{
			if (localPlannerParams.debug >= 1){
				cout<<"Gently left"<<endl;
			}

			if (powerInterruptStarted == true) {
				while (interruptsQueue.size() > 0) {
					interruptsQueue.pop();
				}
				powerInterruptStarted = false;
			}

			globalPlanner->setMotorsVel(localPlannerParams.turnSpeed - localPlannerParams.gentleTurnSpeedDiff, localPlannerParams.turnSpeed);
			if(!turningStarted){
				startTurnTime = std::chrono::high_resolution_clock::now();
			}
			turningStarted = true;
			turnInterrupted = false;
		}
		else{
			if (localPlannerParams.debug >= 1){
				cout<<"Left"<<endl;
			}

			if(powerInterruptStarted == true){
				while(interruptsQueue.size() > 0){
					interruptsQueue.pop();
				}
				powerInterruptStarted = false;
			}

			globalPlanner->setMotorsVel(-localPlannerParams.turnSpeed, localPlannerParams.turnSpeed);
			if(!turningStarted){
				startTurnTime = std::chrono::high_resolution_clock::now();
			}
			turningStarted = true;
			turnInterrupted = false;
		}
	}
	else{
		if (fabs(bestDirLocalMap - localYaw) < localPlannerParams.steeringMargin){
			if (localPlannerParams.debug >= 1){
				cout<<"Straight"<<endl;
			}

			globalPlanner->setMotorsVel(localPlannerParams.normalSpeed, localPlannerParams.normalSpeed);
		}
		else if ((bestDirLocalMap - localYaw > 0) && (bestDirLocalMap - localYaw < localPlannerParams.gentleTurnMargin))	{
			if (localPlannerParams.debug >= 1){
				cout<<"Gently right"<<endl;
			}

			globalPlanner->setMotorsVel(localPlannerParams.turnSpeed, localPlannerParams.turnSpeed - localPlannerParams.gentleTurnSpeedDiff);
		}
		else if ((bestDirLocalMap - localYaw < 0) && (bestDirLocalMap - localYaw > -localPlannerParams.gentleTurnMargin))	{
			if (localPlannerParams.debug >= 1){
				cout<<"Gently left"<<endl;
			}

			globalPlanner->setMotorsVel(localPlannerParams.turnSpeed - localPlannerParams.gentleTurnSpeedDiff, localPlannerParams.turnSpeed);
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

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

#ifndef GLOBALPLANNER_H_
#define GLOBALPLANNER_H_

//STL
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
#include <chrono>
#include <queue>          // std::priority_queue
//OpenCV
#include <opencv2/opencv.hpp>
//Trobot
//#include "../Trobot/include/RobotDrive.h"
//TinyXML
#include <tinyxml.h>
//TAPAS
#include "RobotDrivers/robotDrivers.h"
#include "LocalPlanner.h"

#ifdef TROBOT
#include "../Trobot/include/RobotDrive.h"
#endif

class Robot;
class Debug;
class LocalPlanner;

enum OperationMode{
	Manual,
	Autonomous
};

enum PlanningStage{
	toGoal,
	preciselyToGoal,
	toStart,
	preciselyToStart,
};



class GlobalPlanner {
	friend class Debug;

public:
	class myPQueueComparison {
	public:
		myPQueueComparison() {
		}
		bool operator()(const std::pair<double, int>& lhs,
				const std::pair<double, int>&rhs) const {
			return (lhs.first < rhs.first);
		}
	};

	struct Parameters{
		int runThread;
		double processingFrequency;
		int computeEveryNth;
		int debug;
		double subgoalThreshold, subgoalThreshold2, subgoalAngularThreshold;
		double preciseToGoalMaxTime;
		double changedPlanWaitingTime;
		double changedPlanThreshold;
		double changedPlanDelayTime;
		int sound;
		int runHomologation;
		std::string mapFile;
		double goalLatitude, goalLongitude;
		double startLatitude, startLongitude;
	};

	struct Edge{
		float x1, y1, x2, y2;
		bool isChosen;
		Edge() {}
		Edge(float ix1, float iy1, float ix2, float iy2, float iisChosen):
			x1(ix1),
			y1(iy1),
			x2(ix2),
			y2(iy2), isChosen(iisChosen) {
		}
		bool operator<(const Edge& rhs) const {
			if (x1 < rhs.x1)
				return true;
			else if (fabs(x1 - rhs.x1) < 0.0001 && y1 < rhs.y1)
				return true;
			else if (fabs(x1 - rhs.x1) < 0.0001 && fabs(y1 - rhs.y1) < 0.0001
					&& x2 < rhs.x2)
				return true;
			else if (fabs(x1 - rhs.x1) < 0.0001 && fabs(y1 - rhs.y1) < 0.0001
					&& fabs(x2 - rhs.x2) < 0.0001 && y2 < rhs.y2)
				return true;
			return false;
		}
	};

	struct GlobalPlanInfo {
		std::set<GlobalPlanner::Edge> edges;
		float minX, maxX;
		float minY, maxY;
		float robotX, robotY;
		float goalX, goalY;
		int curEdge;
	};

private:

	// WTF group
	OperationMode currentMode;

	//parent class robot
	Robot* robot;

	//Driver
	Drivers* robotDrive1;
	Drivers* robotDrive2;
	std::mutex driverMtx;
#ifdef TROBOT
	trobot::RobotDrive robotDriveTrobot;
#endif


	// Local Planning
	LocalPlanner* localPlanner;

	// Processing thread
	std::thread globalPlannerThread;

	// Sound thread
	std::thread sayThread;

	// Parameters
	GlobalPlanner::Parameters globalPlannerParams;

	// Processing thread method
	void globalPlannerProcessing();

	// Start the competition
	bool startGlobalPlannerCompetition;

	// Homologation
	void processHomologation();

	// Global planner variables from map
	std::vector<std::pair<double, double>> nodePosition;
	std::vector< std::list<int> > edges;

	// Plan + mutex
	GlobalPlanInfo globalPlanInfo;
	std::mutex mtxGlobalPlan;

	// Stage of the planning
	PlanningStage planningStage;

	// Anti alternative road
	bool weShouldWait;
	std::chrono::high_resolution_clock::time_point waitingStartTime, changePlanDelayStartTime;
	double previousPlanDistance;

	// Goal
	int goalId[2], goalType;
	double goalX, goalY;
	double goalTheta;
	std::chrono::high_resolution_clock::time_point startTime;
	std::list<int> nodesToVisit;
	std::mutex mtxGoalTheta;

	// Robot position
	int startId[2];
	double startIdDist[2];

	// Methods to call
	void readOpenStreetMap(std::string mapName);
	void setLoadedGoal(PlanningStage planningStage);
	void updateRobotPosition(double &robotX, double &robotY, double &theta);
	void findStartingEdge(double robotX, double robotY, int &distType);
	void computeGlobalPlan(double robotX, double robotY, int startType);
	void chooseNextSubGoal(double robotX, double robotY, bool &recomputePlan);

	// Helping methods
	void clearRouteInGlobalPlan();
	// distanceType = 0 (first node), 1 (second node), 2 (edge)
	void findClosestEdge(double X, double Y, int &id1, int &id2, double &minDistance, int& distanceType);
	bool areEdgesEqual(Edge e, Edge f);
	void switchEdge(Edge &e);
	void checkAndCorrectEdgeConvention(Edge &e);
	void updateGoal();
	double computeDistance(std::pair<double, double> a, std::pair<double, double> b);
	int findGoalNodeId(int finalGoalId, std::vector<double> distance);
	void goDirectlyToTarget(double robotX, double robotY, bool& recomputePlan);
	void setGoalDirection(double theta); // in radians
	void setGoalDirectionInDegrees(double theta);

public:
	GlobalPlanner(Robot* irobot, TiXmlElement* settings);
	virtual ~GlobalPlanner();

	void readSettings(TiXmlElement* settings);

	void startCompetition();

	void stopThread();

	float getHeadingToGoal();

#ifdef TROBOT
	//CV_32SC1 2x1: left, right encoder
	cv::Mat getEncoderData(std::chrono::high_resolution_clock::time_point timestamp);

	bool isEncodersOpen();
#endif

	//----------------------MODES OF OPERATION
	void switchMode(OperationMode mode);

	void setMotorsVel(float motLeft, float motRight);

	//----------------------ACCESS TO COMPUTED DATA
	GlobalPlanInfo getGlobalPlan();

	//----------------------MENAGMENT OF GlobalPlanner DEVICES
	//Robots Drive
	void openRobotsDrive(std::string port1, std::string port2);

	void closeRobotsDrive();

	bool isRobotsDriveOpen();
};

#include "../Robot/Robot.h"

#endif /* GLOBALPLANNER_H_ */

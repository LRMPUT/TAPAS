/*
 * GlobalPlanner.h
 *
 * Wanted it to be the global planner containing information about global target and
 * running appropriate local planners
 */

#ifndef GLOBALPLANNER_H_
#define GLOBALPLANNER_H_

//STL
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
#include <chrono>
//OpenCV
#include <opencv2/opencv.hpp>
//Trobot
//#include "../Trobot/include/RobotDrive.h"
//TinyXML
#include <tinyxml.h>
//TAPAS
#include "RobotDrivers/robotDrivers.h"
#include "LocalPlanner.h"


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
	struct Parameters{
		int runThread;
		double processingFrequency;
		int computeEveryNth;
		int debug;
		double subgoalThreshold;
		double preciseToGoalMaxTime;
		int sound;
		int runHomologation;
		std::string mapFile;
		double latitude, longitude;
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

	// Local Planning
	LocalPlanner* localPlanner;

	// Processing thread
	std::thread globalPlannerThread;

	// Sound thread
	std::thread sayThread;

	// Parameters
	GlobalPlanner::Parameters globalPlannerParams;

	// Processing thread method
	void run();

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

	// Goal
	int goalId[2];
	double goalX, goalY;
	double goalTheta;
	std::chrono::high_resolution_clock::time_point startTime;
	std::list<int> nodesToVisit;
	std::mutex mtxGoalTheta;

	// Robot position
	int startingNodeIndex[2];
	double startingNodeDist[2];

	// Methods to call
	void readOpenStreetMap(std::string mapName);
	void setLoadedGoal();
	void updateRobotPosition(double &robotX, double &robotY);
	void findStartingEdge(double robotX, double robotY);
	void computeGlobalPlan(double robotX, double robotY);
	void chooseNextSubGoal(double robotX, double robotY, bool &recomputePlan);

	// Helping methods
	void clearRouteInGlobalPlan();
	void findClosestEdge(double X, double Y, int &id1, int &id2, double &minDistance);
	bool areEdgesEqual(Edge e, Edge f);
	void switchEdge(Edge &e);
	void checkAndCorrectEdgeConvention(Edge &e);
	void updateGoal();

public:
	GlobalPlanner(Robot* irobot, TiXmlElement* settings);
	virtual ~GlobalPlanner();

	void readSettings(TiXmlElement* settings);

	void stopThread();

	void startHomologation();

	float getHeadingToGoal();

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

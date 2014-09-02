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
//OpenCV
#include <opencv2/opencv.hpp>
//Trobot
//#include "../Trobot/include/RobotDrive.h"
//TAPAS
#include "RobotDrivers/robotDrivers.h"
//#include "LocalPlanner.h"


class Robot;
class Debug;
class LocalPlanner;

enum OperationMode{
	Manual,
	Autonomous
};



class GlobalPlanner {
	friend class Debug;

public:
	struct Edge{
		float x1, y1, x2, y2;
		bool isChoosen;
		Edge() {}
		Edge(float ix1, float iy1, float ix2, float iy2, float iisChoosen):
			x1(ix1),
			y1(iy1),
			x2(ix2),
			y2(iy2),
			isChoosen(iisChoosen)
		{}
	};

	struct GlobalPlanInfo {
		std::vector<GlobalPlanner::Edge> edges;
		float minX, maxX;
		float minY, maxY;
		float robotX, robotY;
		int curEdge;
	};

private:
	//parent class robot
	Robot* robot;

	//Driver
	//trobot::RobotDrive* robotDrive;

	Drivers* robotDrive1;
	Drivers* robotDrive2;

	LocalPlanner* localPlanner;

	OperationMode currentMode;

	bool startOperate;

	bool runThread;
	std::thread globalPlannerThread;

	std::mutex driverMtx;

	float currentGoal;

	void run();

	void processHomologation();
public:
	GlobalPlanner(Robot* irobot);
	virtual ~GlobalPlanner();

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

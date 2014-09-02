/*
 * GlobalPlanner.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

//STL
#include <iostream>
#include <chrono>
#include <queue>          // std::priority_queue
//OpenCV
#include <opencv2/opencv.hpp>
//#include "../Trobot/include/RobotDrive.h"
//TAPAS
#include "GlobalPlanner.h"
//#include "../MovementConstraints/MovementConstraints.h"

using namespace cv;
using namespace std;

#define LEFT_CHANNEL 2
#define RIGHT_CHANNEL 1

GlobalPlanner::GlobalPlanner(Robot* irobot) :
		robot(irobot), robotDrive1(NULL), robotDrive2(NULL), startOperate(false) {
	cout << "GlobalPlanner()" << endl;
	runThread = false;
	globalPlannerThread = std::thread(&GlobalPlanner::run, this);

	cout << "End GlobalPlanner()" << endl;
}

GlobalPlanner::~GlobalPlanner() {
	cout << "~GlobalPlanner()" << endl;
	stopThread();
	closeRobotsDrive();
	cout << "End ~GlobalPlanner()" << endl;
}


void GlobalPlanner::run(){
	localPlanner = new LocalPlanner(robot, this);
	while(runThread){
		//readOpenStreetMap("robotourMap.osm");
		if(startOperate){
			//processHomologation();
			localPlanner->localPlanerTest();
		}
		std::chrono::milliseconds duration(50);
		std::this_thread::sleep_for(duration);
	}
}

void GlobalPlanner::processHomologation() {
	cout << "homologation()" << endl;
	if (!isRobotsDriveOpen()) {
		cout << "Robots drive not open, ending homologation" << endl;
		return;
	}
	cout << "Pause" << endl;
	//60 seconds pause
	std::chrono::seconds duration(10);
	std::this_thread::sleep_for(duration);
	cout << "Pause end" << endl;

	bool runForward = true;
	static const float distance = 500000;
	static const int motorsVel = 700;
	float prevDist = robot->getPosImuConstraintsMapCenter().at<float>(0, 3);
	float distRun = 0;
	while (runForward && runThread) {
		Mat constraints = robot->getMovementConstraints();
		Mat posImuMapCenter = robot->getPosImuConstraintsMapCenter();

		bool wayBlocked = false;

		float addX = posImuMapCenter.at<float>(0, 3) / MAP_RASTER_SIZE;
		for (int x = MAP_SIZE / 2; x < MAP_SIZE / 2 + 10; x++) {
			for (int y = MAP_SIZE / 2 - 3; y < MAP_SIZE / 2 + 3; y++) {
				if (x + addX >= 0 && x + addX < MAP_SIZE) {
					if (constraints.at<float>(x + addX, y) > 0.5) {
						wayBlocked = true;
						break;
					}
				}
			}
		}

		cout << "wayBlocked = " << wayBlocked << endl;
		if (wayBlocked == false) {
			cout << "setting " << motorsVel << endl;
			setMotorsVel(motorsVel, motorsVel);
		} else {
			setMotorsVel(1800, -1800);
		}

		float curDist = posImuMapCenter.at<float>(0, 3);
		if (curDist < prevDist) {
			distRun += curDist + (MAP_SIZE / 2 - MAP_MARGIN) * MAP_RASTER_SIZE
					- prevDist;
		} else {
			distRun += curDist - prevDist;
		}
		prevDist = curDist;
		cout << "distRun = " << distRun << endl;
		if (distRun > distance) {
			setMotorsVel(0, 0);
			runForward = false;
		}

		std::chrono::milliseconds duration(200);
		std::this_thread::sleep_for(duration);
	}
	cout << "Position = " << robot->getPosImuConstraintsMapCenter() << endl;
	startOperate = false;
}

class myPQueueComparison {
public:
	myPQueueComparison() {
	}
	bool operator()(const std::pair<double, int>& lhs,
			const std::pair<double, int>&rhs) const {
		return (lhs.first < rhs.second);
	}
};

void GlobalPlanner::readOpenStreetMap(char *mapName) {
	std::cout << "Reading OpenStreetMap" << std::endl;

	TiXmlDocument osmMap(mapName);
	if (!osmMap.LoadFile())
		std::cout << "OSM map was not loaded!" << std::endl;
	;
	TiXmlHandle osmHandle(&osmMap);

	std::map<long int, int> idConversion;

	// Reading nodes
	TiXmlElement* pElem = osmHandle.FirstChildElement().FirstChildElement(
			"node").Element();
	int i = 0;

	for (; pElem; pElem = pElem->NextSiblingElement("node")) {
		const char *pId = pElem->Attribute("id");
		const char *pLat = pElem->Attribute("lat");
		const char *pLon = pElem->Attribute("lon");
		if (pId && pLat && pLon) {
			char *end;
			idConversion.insert(std::make_pair(strtol(pId, &end, 10), i));
//			std::cout<<i << ": OSM - " << atof(pLon) << " " << atof(pLat) << std::endl;
//			std::cout<<i << ": OSM - " << robot->getPosX(atof(pLon)) << " " << robot->getPosY(atof(pLat)) << std::endl;

// Using Lon i Lat -> later change to robot's global map

			nodePosition.push_back(std::make_pair(atof(pLon), atof(pLat)));
		}
		i++;
	}
	std::cout << "Node counter : " << i << std::endl;

	edges.resize(i);
	std::fill(edges.begin(), edges.end(), std::list<int>());

	// Reading ways
	pElem = osmHandle.FirstChildElement().FirstChildElement("way").Element();
	i = 0;
	for (; pElem; pElem = pElem->NextSiblingElement("way")) {
		TiXmlElement* pNode = pElem->FirstChildElement("tag");
		bool footWay = false;

		// Checking way type
		for (; pNode; pNode = pNode->NextSiblingElement("tag")) {
			const char *key = pNode->Attribute("k");
			const char *value = pNode->Attribute("v");
			if (strstr(key, "foot") || strstr(value, "foot")) {
				footWay = true;
				break;
			}
		}

		// If type is correct - add as an edge
		if (footWay) {
			pNode = pElem->FirstChildElement("nd");

			int j = 0;
			// Splitting way into edges
			long int previousId = 0;
			for (; pNode; pNode = pNode->NextSiblingElement("nd")) {
				const char *Id = pNode->Attribute("ref");
				if (Id) {
					char* end;
					std::map<long int, int>::iterator tmp = idConversion.find(
							strtol(Id, &end, 10));
					if (j > 0) {
						edges[tmp->second].push_back(previousId);
						edges[previousId].push_back(tmp->second);
					}
					j++;
					previousId = tmp->second;
				}

			}
			i++;
		}
	}

	std::cout << "Ways found counter : " << i << std::endl;
	std::cout << "Edges counter : " << edges.size() << std::endl;

	// Finding shortest path using Dijkstra
	int startingID = 0;

	// Temporal starting node using the first node with some neighbours
	while (edges[startingID].size() == 0)
		startingID++;

	// Containers for Dijkstra
	std::vector<double> distance(edges.size(), -1.0);
	std::priority_queue<std::pair<double, int>,
			std::vector<std::pair<double, int>>, myPQueueComparison> pqueue;

	// Going through nodes
	distance[startingID] = 0;
	pqueue.push(std::make_pair(0, startingID));
	while (!pqueue.empty()) {
		std::pair<double, int> tmp = pqueue.top();
		pqueue.pop();
		int idToProcess = tmp.second;

		// Looking at neighbours
		for (std::list<int>::iterator listIter = edges[idToProcess].begin();
				listIter != edges[idToProcess].end(); ++listIter) {

			// Computing distance
			double x = nodePosition[idToProcess].first;
			double y = nodePosition[idToProcess].second;
			double x2 = nodePosition[*listIter].first;
			double y2 = nodePosition[*listIter].second;
			double additionalDistance = sqrt(
					(x - x2) * (x - x2) + (y - y2) * (y - y2));

			if (distance[*listIter] == -1
					|| distance[*listIter]
							> distance[idToProcess] + additionalDistance) {
				distance[*listIter] = distance[idToProcess]
						+ additionalDistance;
				pqueue.push(std::make_pair(distance[*listIter], *listIter));
			}
		}

	}

	// Conclusions
	std::cout << "Ended computation from node : " << startingID << std::endl;
	int ile = 0, ogolem = 0;
	for (int i = 0; i < edges.size(); i++) {
		if (edges[i].size() != 0) {
			ogolem++;
			std::cout << "Distance to " << i << " is equal to " << distance[i]
					<< std::endl;
			if (distance[i] > 0)
				ile++;
		}
	}
	std::cout << "Possiblity to access " << ile * 100.0 / ogolem
			<< "% of all nodes" << std::endl;

	findDistances(0, 0);

	while (runThread) {

	}
}

int GlobalPlanner::findDistances(double X, double Y) {

	double minDist = -1;
	int save1, save2;
	for (int i = 0; i < edges.size(); i++) {
		for (list<int>::iterator lIt = edges[i].begin(); lIt != edges[i].end();
				++lIt) {
			std::pair<double, double> node1 = nodePosition[i], node2 =
					nodePosition[*lIt];

			double A = node2.second - node1.second;
			double B = node1.first - node2.first;
			double C = node1.second * node2.first - node1.first * node2.second;

			double d = fabs(A * X + B * Y + C) / sqrt(A * A + B * B);

			if (d < minDist || minDist < 0) {
				minDist = d;
				save1 = i;
				save2 = *lIt;
			}
		}
	}
	std::cout << "MinDist - " << minDist << std::endl;

	std::pair<double, double> node1 = nodePosition[save1], node2 =
			nodePosition[save2];
	std::cout << "Compare - " << node1.first << " " << node1.second << " " << X
			<< " | " << node2.first << " " << node2.second << " " << Y
			<< std::endl;

	double distance1 = sqrt(
			pow(node1.first - X, 2) + pow(node1.second - Y, 2)
					- minDist * minDist / 4);
	double distance2 = sqrt(
			pow(node2.first - X, 2) + pow(node2.second - Y, 2)
					- minDist * minDist / 4);

	std::cout << "The closest edge is the one between " << save1 << " and "
			<< save2 << std::endl;
	std::cout << "Distances " << distance1 << " and " << distance2 << std::endl;

	return 0;
}

void GlobalPlanner::stopThread() {
	runThread = false;
	if (globalPlannerThread.joinable()) {
		globalPlannerThread.join();
	}
}

void GlobalPlanner::startHomologation() {
	startOperate = true;
}

//----------------------MODES OF OPERATION
void GlobalPlanner::switchMode(OperationMode mode) {
	currentMode = mode;
	setMotorsVel(0, 0);
}

void GlobalPlanner::setMotorsVel(float motLeft, float motRight) {

	std::unique_lock < std::mutex > lck(driverMtx);
	if (robotDrive1 != NULL && robotDrive2 != NULL) {
		robotDrive1->exitSafeStart();
		robotDrive2->exitSafeStart();

		robotDrive1->setMotorSpeed(motLeft);
		robotDrive2->setMotorSpeed(-motRight);
	}
	lck.unlock();
}

//----------------------ACCESS TO COMPUTED DATA
GlobalPlanner::GlobalPlanInfo GlobalPlanner::getGlobalPlan()
{
	GlobalPlanInfo x;
	return x;
}

//----------------------MENAGMENT OF GlobalPlanner DEVICES
//Robots Drive
void GlobalPlanner::openRobotsDrive(std::string port1, std::string port2) {
	//closeRobotsDrive();
	//robotDrive = new RobotDrive(port);
	cout << "Left: " << port1 << ", right: " << port2 << endl;

	closeRobotsDrive();
	std::unique_lock < std::mutex > lck(driverMtx);
	robotDrive1 = new Drivers(115200, port1);
	robotDrive2 = new Drivers(115200, port2);
	lck.unlock();

	setMotorsVel(0, 0);
}

void GlobalPlanner::closeRobotsDrive() {
	cout << "closeRobotsDrive()" << endl;
	std::unique_lock < std::mutex > lck(driverMtx);
	if (robotDrive1 != NULL) {
		robotDrive1->exitSafeStart();
		robotDrive1->setMotorSpeed(0);
		delete robotDrive1;
		robotDrive1 = NULL;
	}
	if (robotDrive2 != NULL) {
		robotDrive2->exitSafeStart();
		robotDrive2->setMotorSpeed(0);
		delete robotDrive2;
		robotDrive2 = NULL;
	}
	lck.unlock();
	cout << "End closeRobotsDrive()" << endl;
}

bool GlobalPlanner::isRobotsDriveOpen() {
	return (robotDrive1 != NULL) && (robotDrive2 != NULL);
}

float GlobalPlanner::getHeadingToGoal(){

	// indicate: go straight ahead
	currentGoal = 90.0;

	return currentGoal;
}


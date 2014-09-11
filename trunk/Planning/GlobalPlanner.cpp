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

#define ROBOT_DRIVE_MAX 3200

class myPQueueComparison {
public:
	myPQueueComparison() {
	}
	bool operator()(const std::pair<double, int>& lhs,
			const std::pair<double, int>&rhs) const {
		return (lhs.first < rhs.first);
	}
};

GlobalPlanner::GlobalPlanner(Robot* irobot, TiXmlElement* settings) :
		robot(irobot), robotDrive1(NULL), robotDrive2(NULL) {
	cout << "GlobalPlanner()" << endl;
	readSettings(settings);

	localPlanner = new LocalPlanner(robot, this, settings);
	globalPlannerThread = std::thread(&GlobalPlanner::run, this);

	cout << "End GlobalPlanner()" << endl;
}


GlobalPlanner::~GlobalPlanner() {
	cout << "~GlobalPlanner()" << endl;
	stopThread();
	closeRobotsDrive();
	cout << "End ~GlobalPlanner()" << endl;
}

// Reading settings

void GlobalPlanner::readSettings(TiXmlElement* settings)
{
	TiXmlElement* pGlobalPlanner;
	pGlobalPlanner = settings->FirstChildElement("GlobalPlanner");

	if (pGlobalPlanner->QueryIntAttribute("runThread", &globalPlannerParams.runThread)
			!= TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner Thread";
	}
	if (pGlobalPlanner->QueryDoubleAttribute("processingFrequency",
			&globalPlannerParams.processingFrequency) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner processing frequency";
	}
	if (pGlobalPlanner->QueryIntAttribute("debug", &globalPlannerParams.debug)
			!= TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner debug";
	}
	if (pGlobalPlanner->QueryIntAttribute("runHomologation", &globalPlannerParams.runHomologation)
			!= TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner runHomologation";
	}

	if (pGlobalPlanner->QueryStringAttribute("mapFile", &globalPlannerParams.mapFile)
			!= TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner mapFile";
	}

	if (pGlobalPlanner->QueryDoubleAttribute("latitude",
			&globalPlannerParams.latitude) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner latitude";
	}

	if (pGlobalPlanner->QueryDoubleAttribute("longitude",
			&globalPlannerParams.longitude) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner longitude";
	}


	printf("GlobalPlanner -- runThread: %d\n", globalPlannerParams.runThread);
	printf("GlobalPlanner -- debug: %d\n", globalPlannerParams.debug);
	printf("GlobalPlanner -- runhomologation: %d\n", globalPlannerParams.runHomologation);
	printf("GlobalPlanner -- mapFile: %s\n", globalPlannerParams.mapFile.c_str());
	printf("GlobalPlanner -- goal latitude: %f \n", globalPlannerParams.latitude);
	printf("GlobalPlanner -- goal longitude: %f \n", globalPlannerParams.longitude);
}


// Main processing thread
void GlobalPlanner::run() {
//	localPlanner->startLocalPlanner();

	if (globalPlannerParams.debug == 1)
		std::cout<<"GlobalPlanner: Waiting for GPS set zero" <<std::endl;

	while ((robot->isSetZero() == false || robot->gpsGetFixStatus() == 1)
			&& globalPlannerParams.runThread)
	{
		usleep(200);
	}

	if (globalPlannerParams.debug == 1)
		std::cout<<"GlobalPlanner: We have fix and zero position" <<std::endl;

	if(globalPlannerParams.runThread){
		// Read the map of the tournament
		readOpenStreetMap(globalPlannerParams.mapFile);

		// Set the goal in the map
		setGoal();
	}
	while (globalPlannerParams.runThread) {

			// Where are we ?
			updateRobotPosition();

			// Where are we in the map
			findStartingEdge(robotX, robotY);

			// Compute the route to follow
			computeGlobalPlan();

			// Let's compute the next target
//			chooseNextSubGoal();

			// Let's update out subgoal as current destination
//			updateHeadingGoal();

			break;

		std::chrono::milliseconds duration(int(1000.0/globalPlannerParams.processingFrequency));
		std::this_thread::sleep_for(duration);
	}
//	localPlanner->stopLocalPlanner();

}


// Methods called in processing thread

void GlobalPlanner::readOpenStreetMap(std::string mapName) {
	if (globalPlannerParams.debug == 1)
		std::cout << "GlobalPlanner: Reading OpenStreetMap" << std::endl;

	TiXmlDocument osmMap(mapName);
	if (!osmMap.LoadFile())
		std::cout << "GlobalPlanner: OSM map was not loaded!" << std::endl;
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

			// We need to convert from OSM's gps coordinates to Nmea format
			double lon = GPS::decimal2Nmea(atof(pLon)*100);
			double lat = GPS::decimal2Nmea(atof(pLat)*100);

			// Adding the node
			nodePosition.push_back(
					std::make_pair(robot->getPosX(lat)/1000,
							robot->getPosY(lon)/1000));
		}
		i++;
	}

	// List of edges
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

			// Is is a way we can walk on? Can add here more way tags if needed
			if (strstr(key, "foot") || strstr(value, "foot") || strstr(value, "service")) {
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

	if (globalPlannerParams.debug == 1)
		std::cout << "GlobalPlanner: Found " << edges.size() << " in read mapfile"<< std::endl;


	// Adding edges to the global plan
	std::unique_lock<std::mutex> lckGlobalPlan(mtxGlobalPlan);
	globalPlanInfo.robotX = robotX;
	globalPlanInfo.robotY = robotY;
	double minX = robotX, maxX = robotX, minY = robotY, maxY = robotY;

	for (int i = 0; i < edges.size(); i++) {
		for (list<int>::iterator lIt = edges[i].begin(); lIt != edges[i].end();
				++lIt) {

			// Node positions of both ends of the edge
			std::pair<double, double> node1 = nodePosition[i], node2 =
					nodePosition[*lIt];

			// Creating the edge
			Edge edge;
			edge.isChosen = false;

			edge.x1 = node1.first;
			edge.y1 = node1.second;
			edge.x2 = node2.first;
			edge.y2 = node2.second;

			checkAndCorrectEdgeConvention(edge);
			globalPlanInfo.edges.insert(edge);

			// Finding the minimal and maximal coordinates
			double t = std::min(edge.x1, edge.x2);
			minX = std::min(minX, t);
			t = std::max(edge.x1, edge.x2);
			maxX = std::max(maxX, t);
			t = std::min(edge.y1, edge.y2);
			minY = std::min(minY, t);
			t = std::max(edge.y1, edge.y2);
			maxY = std::max(maxY, t);
		}

	}

	if (globalPlannerParams.debug == 1)
	{
		std::cout << "GlobalPlanner: mixX, maxX, minY, maxY : " << minX << ", " << maxX << ", " << minY << ", "
				<< maxY << std::endl;

		std::cout << "GlobalPlanner: robotX, robotY : " << robotX << ", " << robotY <<endl;
	}

	globalPlanInfo.minX = minX;
	globalPlanInfo.maxX = maxX;
	globalPlanInfo.minY = minY;
	globalPlanInfo.maxY = maxY;
	lckGlobalPlan.unlock();


	if (globalPlannerParams.debug == 1)
		std::cout<<"Global Planner: end readosmMap"<<std::endl;
}

void GlobalPlanner::setGoal() {

	// Recomputing from decimal coordinates to Nmea
	double lon = GPS::decimal2Nmea(globalPlannerParams.longitude * 100);
	double lat = GPS::decimal2Nmea(globalPlannerParams.latitude * 100);
	if (globalPlannerParams.debug == 1)
		std::cout << "GlobalPlanner: goal longitude and latitude: " << lat / 100 << " "
			<< lon / 100 << std::endl;
	goalX = robot->getPosX(lat)/1000;
	goalY = robot->getPosY(lon)/1000;


	// Updating global plan
	std::unique_lock<std::mutex> lckGlobalPlan(mtxGlobalPlan);
		globalPlanInfo.goalX = goalX;
		globalPlanInfo.goalY = goalY;
	lckGlobalPlan.unlock();


	// Finding the edge that is the closest to the goal
	double dist;
	findClosestEdge(goalX, goalY, goalId[0], goalId[1], dist);
}

void GlobalPlanner::updateRobotPosition() {

	cv::Mat robotPosition = robot->getEstimatedPosition();
	robotX = robotPosition.at<double>(0);
	robotY = robotPosition.at<double>(1);
	robotTheta = robotPosition.at<double>(2);

	std::unique_lock<std::mutex> lckGlobalPlan(mtxGlobalPlan);
		globalPlanInfo.robotX = robotX;
		globalPlanInfo.robotY = robotY;
	lckGlobalPlan.unlock();

	if (globalPlannerParams.debug == 1)
		std::cout << "Global Planner : robot position : " << robotX << " " <<robotY<<std::endl;
}

void GlobalPlanner::findStartingEdge(double X, double Y) {

	// Looking for the edges closest the the starting X and Y
	double minDist;
	findClosestEdge(X, Y, startingNodeIndex[0], startingNodeIndex[1], minDist);

	// We have the two nodes that are connected with the closest edge
	for (int i=0;i<2;i++)
	{
		std::pair<double, double> node = nodePosition[startingNodeIndex[i]];

		// We compute the distance to this node
		startingNodeDist[i] = sqrt( pow(node.first - X, 2) + pow(node.second - Y, 2) );

		if (globalPlannerParams.debug == 1)
			std::cout << "The closest edge's node is the one between "
					<< startingNodeIndex[i] << " with distance = " << startingNodeDist[i] << std::endl;

	}

	// We need to update the position of the closest edge in the map
	std::unique_lock < std::mutex > lckGlobalPlan(mtxGlobalPlan);

	Edge e;
	e.x1 = nodePosition[startingNodeIndex[0]].first;
	e.y1 = nodePosition[startingNodeIndex[0]].second;
	e.x2 = nodePosition[startingNodeIndex[1]].first;
	e.y2 = nodePosition[startingNodeIndex[1]].second;

	// Lets look for the edge in the visualization
	int i = 0;
	std::set<GlobalPlanner::Edge>::iterator it = globalPlanInfo.edges.begin();
	for (; it != globalPlanInfo.edges.end(); i++) {

		// We found our edge
		if (areEdgesEqual(e, *it)) {
			globalPlanInfo.curEdge = i;
		}
		// We also invalidate the previously found route
//		if (it->isChosen == true) {
//			GlobalPlanner::Edge tmp = *it;
//			it = globalPlanInfo.edges.erase(it);
//			tmp.isChosen = false;
//			globalPlanInfo.edges.insert(tmp);
//		}
//		else
			++it;
	}

	lckGlobalPlan.unlock();

	if (globalPlannerParams.debug == 1)
		std::cout << "GlobalPlanner : Finished finding current edge " << std::endl;

}

void GlobalPlanner::computeGlobalPlan() {
	// Finding shortest path using Dijkstra

	// Containers for Dijkstra
	std::vector<double> distance(edges.size(), -1.0);
	std::vector<int> previous(edges.size(), -1.0);
	std::priority_queue<std::pair<double, int>,
			std::vector<std::pair<double, int>>, myPQueueComparison> pqueue;

	// Going through nodes
	distance[ startingNodeIndex[0] ] = startingNodeDist[0];
	distance[ startingNodeIndex[1] ] = startingNodeDist[1];

	previous[ startingNodeIndex[0] ] = -2.0;
	previous[ startingNodeIndex[1] ] = -2.0;

	pqueue.push(std::make_pair(startingNodeDist[0],  startingNodeIndex[0]));
	pqueue.push(std::make_pair(startingNodeDist[1],  startingNodeIndex[1]));

	// We process the graph
	while (!pqueue.empty()) {

		// Currently first in the queue
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

			// If it is a new node in processing or we found shorter path
			if ( fabs(distance[*listIter] + 1) < 0.0000001
					|| distance[*listIter]
							> distance[idToProcess] + additionalDistance) {
				distance[*listIter] = distance[idToProcess]
						+ additionalDistance;
				previous[*listIter] = idToProcess;

				// We add as a new node to process
				pqueue.push(std::make_pair(distance[*listIter], *listIter));
			}
		}

	}

	// Conclusions
	if (globalPlannerParams.debug == 1)
	{
		std::cout << "Global Planner : Printing ids from backward to get to id="
				<< goalId[0] << " or " << goalId[1] << " from ids="
				<< startingNodeIndex[0] << " or " << startingNodeIndex[1]
				<< std::endl;

		std::cout<<std::endl<<" GOAL DISTANCE : "  << distance[goalId[0]] << " or " << distance[goalId[1]] << std::endl;
	}

	// Let's check which node of the final edge is closer to us
	int finalGoalId = goalId[0];
	if ( distance[finalGoalId] > distance[goalId[1]] )
		finalGoalId  = goalId[1];

	// Print the route until we are in our position
	int i = finalGoalId;
	while( i != startingNodeIndex[0] && i != startingNodeIndex[1] )
	{
		int k = previous[i];

		// We go using this edge
		Edge e;
		e.x1 = nodePosition[i].first, e.y1 = nodePosition[i].second;
		e.x2 = nodePosition[k].first, e.y2 = nodePosition[k].second;

		std::unique_lock < std::mutex > lckGlobalPlan(mtxGlobalPlan);
		std::set<GlobalPlanner::Edge>::iterator it = globalPlanInfo.edges.begin();

		// We look for this edge
		bool found = false;
		for (; it != globalPlanInfo.edges.end(); i++) {
			if ( areEdgesEqual(e, *it)) {
				// We mark them as chosen in our route
				GlobalPlanner::Edge tmp = *it;
				it = globalPlanInfo.edges.erase(it);
				tmp.isChosen = true;
				std::cout<<"Global Planner:: Edge sizes 1: " << globalPlanInfo.edges.size() << std::endl;
				globalPlanInfo.edges.insert(tmp);
				std::cout<<"Global Planner:: Edge sizes 2: " << globalPlanInfo.edges.size() << std::endl;
				found = true;
			} else
				++it;
		}
		if (globalPlannerParams.debug == 1)
		{
			std::cout<<"Global Planner:: Edge sizes : " << globalPlanInfo.edges.size() << std::endl;
		}

		lckGlobalPlan.unlock();

		if (globalPlannerParams.debug == 1)
		{
			if (found == false)
				std::cout<<"Global Planner: NIE ZNALEZIONO KRAWEDZI z drogi w grafie " << std::endl;
		}
		// Let's move using the route
		i = k;
	}

}

// Helping methods

void GlobalPlanner::findClosestEdge(double X, double Y, int &id1, int &id2, double &minDistance) {

	minDistance = -1;
	// For all nodes
	for (int i = 0; i < edges.size(); i++) {

		// For all neighbours
		for (list<int>::iterator lIt = edges[i].begin(); lIt != edges[i].end();
				++lIt) {

			// Edge
			std::pair<double, double> node1 = nodePosition[i], node2 =
					nodePosition[*lIt];

			// Distance to the infinite line
			double A = node2.second - node1.second;
			double B = node1.first - node2.first;
			double C = node1.second * node2.first - node1.first * node2.second;

			double d = fabs(A * X + B * Y + C) / sqrt(A * A + B * B);

			// Helping variables
			double x1 = node1.first, y1 = node1.second;
			double x2 = node2.first, y2 = node2.second;

			// Check the dot products -> because we look for distance to section
			double a_x = x1 - X, a_y = y1 - Y;
			double b_x = x1 - x2, b_y = y1 - y2;
			double c_x = x2 - X, c_y = y2 - Y;

			double ab = a_x * b_x + a_y * b_y;
			double cb = -c_x * b_x - c_y * b_y;

			// The distance is computed to the 1st vertex
			if (ab <= 0 && cb >= 0)
			{
				d = sqrt( (x1 - X)*(x1 - X) + (y1 - Y)*(y1 - Y) );
			}
			// ... and to the second vertex
			else if (ab >= 0 && cb <= 0)
			{
				d = sqrt( (x2 - X)*(x2 - X) + (y2 - Y)*(y2 - Y) );
			}

			// Let's see if this edge is the closest one to the currently check position
			if (d < minDistance || minDistance < 0) {
				minDistance = d;
				id1 = i;
				id2 = *lIt;
			}
		}
	}
	if (globalPlannerParams.debug == 1)
		std::cout << "Global Planner: the ids of the closest edge are : " << id1
				<< " and " << id2 << " and the distance is " << minDistance
				<< " meters"<< std::endl;
}

bool GlobalPlanner::areEdgesEqual(Edge e, Edge f)
{
	checkAndCorrectEdgeConvention(e);
	checkAndCorrectEdgeConvention(f);

	double d = fabs(e.x1 - f.x1) + fabs(e.y1 - f.y1) + fabs(e.x2 - f.x2) + fabs(e.y2 - f.y2);
	if ( d > 0.000001)
		return false;
	return true;
}

void GlobalPlanner::switchEdge(Edge &e)
{
	double a = e.x1;
	e.x1 = e.x2;
	e.x2 = a;

	a = e.y1;
	e.y1 = e.y2;
	e.y2 = a;
}

void GlobalPlanner::checkAndCorrectEdgeConvention(Edge &e)
{
	if (e.x1 < e.x2)
		switchEdge(e);
	else if ( fabs(e.x1 - e.x2) < 0.000001 && e.y1 < e.y2 )
		switchEdge(e);
}

// Homologation

void GlobalPlanner::processHomologation() {

	/// MISSING START OPERATE - removed. Should use parameters
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
	while (runForward && globalPlannerParams.runThread) {
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
}

void GlobalPlanner::startHomologation() {
	localPlanner->startLocalPlanner();
}

// Stop thread

void GlobalPlanner::stopThread() {
	globalPlannerParams.runThread = 0;
	if (globalPlannerThread.joinable()) {
		globalPlannerThread.join();
	}
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

		robotDrive1->setMotorSpeed(max(min((int)(motLeft*ROBOT_DRIVE_MAX/100), ROBOT_DRIVE_MAX), -ROBOT_DRIVE_MAX));
		robotDrive2->setMotorSpeed(max(min((int)(-motRight*ROBOT_DRIVE_MAX/100), ROBOT_DRIVE_MAX), -ROBOT_DRIVE_MAX));
	}
	lck.unlock();
}

//----------------------ACCESS TO COMPUTED DATA
GlobalPlanner::GlobalPlanInfo GlobalPlanner::getGlobalPlan() {
//	x.minX = 0;
//	x.maxX = 5;
//	x.minY = 0;
//	x.maxY = 5;
//	x.robotX = 2;
//	x.robotY = 2;
//	x.curEdge = 0;
//
//	GlobalPlanner::Edge tmpEdge;
//	tmpEdge.isChoosen = 1;
//	tmpEdge.x1 = 0.5;
//	tmpEdge.y1 = 0.5;
//	tmpEdge.x2 = 2.5;
//	tmpEdge.y2 = 2.5;
//
//	x.edges.push_back(tmpEdge);
	std::unique_lock<std::mutex> lckGlobalPlan(mtxGlobalPlan);
	GlobalPlanInfo plan = globalPlanInfo;
	lckGlobalPlan.unlock();

	return plan;
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

float GlobalPlanner::getHeadingToGoal() {

	// indicate: go straight ahead
	float currentGoal = 90.0;

	return currentGoal;
}


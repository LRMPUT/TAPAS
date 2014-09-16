/*
 * GlobalPlanner.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

//STL
#include <iostream>
#include <chrono>
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

GlobalPlanner::GlobalPlanner(Robot* irobot, TiXmlElement* settings) :
		robot(irobot), robotDrive1(NULL), robotDrive2(NULL), goalTheta(0.0), planningStage(
				toGoal), weShouldWait(false), previousPlanDistance(-1.0) {
	cout << "GlobalPlanner()" << endl;
	readSettings(settings);

	localPlanner = new LocalPlanner(robot, this, settings);

	changePlanDelayStartTime = std::chrono::high_resolution_clock::now();

	globalPlannerThread = std::thread(&GlobalPlanner::globalPlannerProcessing,
			this);

	cout << "End GlobalPlanner()" << endl;
}

GlobalPlanner::~GlobalPlanner() {
	cout << "~GlobalPlanner()" << endl;
	delete localPlanner;
	closeRobotsDrive();
	stopThread();
	cout << "End ~GlobalPlanner()" << endl;
}

// Reading settings

void GlobalPlanner::readSettings(TiXmlElement* settings) {
	TiXmlElement* pGlobalPlanner;
	pGlobalPlanner = settings->FirstChildElement("GlobalPlanner");

	if (pGlobalPlanner->QueryIntAttribute("runThread",
			&globalPlannerParams.runThread) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner Thread";
	}
	if (pGlobalPlanner->QueryDoubleAttribute("processingFrequency",
			&globalPlannerParams.processingFrequency) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner processing frequency";
	}
	if (pGlobalPlanner->QueryIntAttribute("computeEveryNth",
			&globalPlannerParams.computeEveryNth) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner computeEveryNth";
	}
	if (pGlobalPlanner->QueryIntAttribute("debug", &globalPlannerParams.debug)
			!= TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner debug";
	}
	if (pGlobalPlanner->QueryDoubleAttribute("subgoalThreshold",
			&globalPlannerParams.subgoalThreshold) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner subgoalThreshold";
	}
	if (pGlobalPlanner->QueryDoubleAttribute("subgoalThreshold2",
			&globalPlannerParams.subgoalThreshold2) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner subgoalThreshold2";
	}
	if (pGlobalPlanner->QueryDoubleAttribute("subgoalAngularThreshold",
			&globalPlannerParams.subgoalAngularThreshold) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner subgoalAngularThreshold";
	}
	if (pGlobalPlanner->QueryDoubleAttribute("preciseToGoalMaxTime",
			&globalPlannerParams.preciseToGoalMaxTime) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner preciseToGoalMaxTime";
	}
	if (pGlobalPlanner->QueryDoubleAttribute("changedPlanWaitingTime",
			&globalPlannerParams.changedPlanWaitingTime) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner changedPlanWaitingTime";
	}
	if (pGlobalPlanner->QueryDoubleAttribute("changedPlanThreshold",
			&globalPlannerParams.changedPlanThreshold) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner changedPlanThreshold";
	}
	if (pGlobalPlanner->QueryDoubleAttribute("changedPlanDelayTime",
			&globalPlannerParams.changedPlanThreshold) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner changedPlanDelayTime";
	}
	if (pGlobalPlanner->QueryIntAttribute("sound", &globalPlannerParams.sound)
			!= TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner sound";
	}
	if (pGlobalPlanner->QueryIntAttribute("runHomologation",
			&globalPlannerParams.runHomologation) != TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner runHomologation";
	}

	if (pGlobalPlanner->QueryStringAttribute("mapFile",
			&globalPlannerParams.mapFile) != TIXML_SUCCESS) {
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
	printf("GlobalPlanner -- processing frequency: %f\n",
			globalPlannerParams.processingFrequency);
	printf("GlobalPlanner -- computeEveryNth: %d\n",
			globalPlannerParams.computeEveryNth);
	printf("GlobalPlanner -- debug: %d\n", globalPlannerParams.debug);
	printf("GlobalPlanner -- subgoal threshold: %f\n",
			globalPlannerParams.subgoalThreshold);
	printf("GlobalPlanner -- subgoal threshold2: %f\n",
			globalPlannerParams.subgoalThreshold2);
	printf("GlobalPlanner -- subgoal angular threshold: %f\n",
			globalPlannerParams.subgoalAngularThreshold);
	printf("GlobalPlanner -- preciseToGoalMaxTime: %f\n",
			globalPlannerParams.preciseToGoalMaxTime);
	printf("GlobalPlanner -- changedPlanWaitingTime: %f\n",
			globalPlannerParams.changedPlanWaitingTime);
	printf("GlobalPlanner -- changedPlanThreshold: %f\n",
			globalPlannerParams.changedPlanThreshold);
	printf("GlobalPlanner -- sound: %d\n", globalPlannerParams.sound);
	printf("GlobalPlanner -- runhomologation: %d\n",
			globalPlannerParams.runHomologation);
	printf("GlobalPlanner -- mapFile: %s\n",
			globalPlannerParams.mapFile.c_str());
	printf("GlobalPlanner -- goal latitude: %f \n",
			globalPlannerParams.latitude);
	printf("GlobalPlanner -- goal longitude: %f \n",
			globalPlannerParams.longitude);
}

// Main processing thread
void GlobalPlanner::globalPlannerProcessing() {

	if ( globalPlannerParams.runHomologation == 1 )
	{
		usleep(200000); // TODO
		while(!robot->isImuDataValid())
			usleep(200000);

		localPlanner->startLocalPlanner();
		while (globalPlannerParams.runThread)
		{
			std::chrono::high_resolution_clock::time_point timestamp;
			//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
			cv::Mat imu = robot->getImuData(timestamp);
			setGoalDirectionInDegrees(imu.at<float>(2,3));
			usleep(200000);
		}
	} else {
		while ((!robot->isGpsOpen() || robot->isSetZero() == false
				|| robot->gpsGetFixStatus() == 1)
				&& globalPlannerParams.runThread) {
			usleep(200);
		}
		while (!robot->isGpsDataValid() && globalPlannerParams.runThread) {
			usleep(200);
		}
		if (globalPlannerParams.debug == 1)
			std::cout << "Global Planner : We have fix and zero position"
					<< std::endl;

		if (globalPlannerParams.runThread) {
			// Read the map of the tournament
			readOpenStreetMap(globalPlannerParams.mapFile);

			// Set the goal in the map
			setLoadedGoal();
		}
		localPlanner->setNormalSpeed();
		localPlanner->startLocalPlanner();
		int loopTimeCounter = 0;
		bool recomputePlan = false;
		int startType = 0;
		while (globalPlannerParams.runThread) {
			std::cout << std::endl << "Global Planner : New iteration"
					<< std::endl << std::endl;

			// Where are we ?
			double robotX, robotY, theta;
			updateRobotPosition(robotX, robotY, theta);

			// Where are we in the map
			findStartingEdge(robotX, robotY, startType);

			// We compute new global plan
			if (globalPlannerParams.computeEveryNth == 1 || recomputePlan
					|| loopTimeCounter % globalPlannerParams.computeEveryNth
							== 0) {
				loopTimeCounter = loopTimeCounter
						% globalPlannerParams.computeEveryNth;
				// Compute the route to follow
				computeGlobalPlan(robotX, robotY, startType);
				// We recomputed the plan
				recomputePlan = false;

			}

			// Let's compute the next target
			chooseNextSubGoal(robotX, robotY, recomputePlan);

			// Let's update out subgoal as current destination
			//			updateHeadingGoal();

			std::chrono::milliseconds duration(
					int(1000.0 / globalPlannerParams.processingFrequency));
			std::this_thread::sleep_for(duration);
			loopTimeCounter++;
		}
		localPlanner->stopLocalPlanner();
	}
}

// Methods called in processing thread

void GlobalPlanner::readOpenStreetMap(std::string mapName) {

	if (globalPlannerParams.debug == 1)
		std::cout << "Global Planner : Reading OpenStreetMap" << std::endl;

	TiXmlDocument osmMap(mapName);
	if (!osmMap.LoadFile())
		std::cout << "Global Planner : OSM map was not loaded!" << std::endl;
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
			double lon = GPS::decimal2Nmea(atof(pLon) * 100);
			double lat = GPS::decimal2Nmea(atof(pLat) * 100);

			// Adding the node
			nodePosition.push_back(
					std::make_pair(robot->getPosX(lat) / 1000,
							robot->getPosY(lon) / 1000));
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
			if (strstr(key, "foot") || strstr(value, "foot")
					|| strstr(value, "service") || strstr(value, "tertiary")
					|| strstr(value, "residential") || strstr(value, "track")
					|| strstr(value, "cycleway")) {
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
		std::cout << "Global Planner : Found " << edges.size()
				<< " in read mapfile" << std::endl;

	// Adding edges to the global plan
	std::unique_lock < std::mutex > lckGlobalPlan(mtxGlobalPlan);
	globalPlanInfo.robotX = 0;
	globalPlanInfo.robotY = 0;
	double minX = 0, maxX = 0, minY = 0, maxY = 0;

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

	if (globalPlannerParams.debug == 1) {
		std::cout << "Global Planner : mixX, maxX, minY, maxY : " << minX
				<< ", " << maxX << ", " << minY << ", " << maxY << std::endl;
	}

	globalPlanInfo.minX = minX;
	globalPlanInfo.maxX = maxX;
	globalPlanInfo.minY = minY;
	globalPlanInfo.maxY = maxY;
	lckGlobalPlan.unlock();

	if (globalPlannerParams.debug == 1)
		std::cout << "Global Planner: end readosmMap" << std::endl;
}

void GlobalPlanner::updateGoal() {
	// Updating global plan
	std::unique_lock < std::mutex > lckGlobalPlan(mtxGlobalPlan);
	globalPlanInfo.goalX = goalX;
	globalPlanInfo.goalY = goalY;
	lckGlobalPlan.unlock();
	// Finding the edge that is the closest to the goal
	double dist;
	findClosestEdge(goalX, goalY, goalId[0], goalId[1], dist, goalType);
}

void GlobalPlanner::setLoadedGoal() {

	// Recomputing from decimal coordinates to Nmea
	double lon = GPS::decimal2Nmea(globalPlannerParams.longitude * 100);
	double lat = GPS::decimal2Nmea(globalPlannerParams.latitude * 100);
	if (globalPlannerParams.debug == 1)
		std::cout << "Global Planner : goal longitude and latitude: "
				<< lat / 100 << " " << lon / 100 << std::endl;
	goalX = robot->getPosX(lat) / 1000;
	goalY = robot->getPosY(lon) / 1000;

	// Updating global plan
	updateGoal();
}

void GlobalPlanner::updateRobotPosition(double &robotX, double &robotY,
		double &theta) {

	cv::Mat robotPosition = robot->getEstimatedPosition();
	robotX = robotPosition.at<double>(0);
	robotY = robotPosition.at<double>(1);
	theta = robotPosition.at<double>(2);

	std::unique_lock < std::mutex > lckGlobalPlan(mtxGlobalPlan);
	globalPlanInfo.robotX = robotX;
	globalPlanInfo.robotY = robotY;

	globalPlanInfo.minX = std::min(globalPlanInfo.minX, (float) robotX);
	globalPlanInfo.maxX = std::max(globalPlanInfo.maxX, (float) robotX);
	globalPlanInfo.minY = std::min(globalPlanInfo.minY, (float) robotY);
	globalPlanInfo.maxY = std::max(globalPlanInfo.maxY, (float) robotY);

	lckGlobalPlan.unlock();

	if (globalPlannerParams.debug == 1)
		std::cout << "Global Planner : robot position : " << robotX << " "
				<< robotY << std::endl;
}

void GlobalPlanner::findStartingEdge(double X, double Y, int &distType) {

	// Looking for the edges closest the the starting X and Y
	double minDist;
	findClosestEdge(X, Y, startId[0], startId[1], minDist, distType);

	// We go to edge
	int iStart = 0, iEnd = 2;
	// We go to either of nodes (NOT EDGE)
	if (distType == 0)
		iEnd = 1;
	else if (distType == 1)
		iStart = 1;

	if (globalPlannerParams.debug == 1)
		std::cout << "Global Planner : distance type = " << distType
				<< std::endl;

	// We have the two nodes that are connected with the closest edge
	for (int i = 0; i < 2; i++) {
		std::pair<double, double> node = nodePosition[startId[i]];

		// We compute the distance to this node
		startIdDist[i] = sqrt(pow(node.first - X, 2) + pow(node.second - Y, 2));

		if (globalPlannerParams.debug == 1)
			std::cout
					<< "Global Planner : The closest edge's node is the one between "
					<< startId[i] << " with distance = " << startIdDist[i]
					<< std::endl;

	}

	// We need to update the position of the closest edge in the map
	std::unique_lock < std::mutex > lckGlobalPlan(mtxGlobalPlan);

	Edge e;
	e.x1 = nodePosition[startId[0]].first;
	e.y1 = nodePosition[startId[0]].second;
	e.x2 = nodePosition[startId[1]].first;
	e.y2 = nodePosition[startId[1]].second;

	// Lets look for the edge in the visualization
	int i = 0;
	std::set<GlobalPlanner::Edge>::iterator it = globalPlanInfo.edges.begin();
	for (; it != globalPlanInfo.edges.end(); i++, ++it) {

		// We found our edge
		if (areEdgesEqual(e, *it)) {
			globalPlanInfo.curEdge = i;
			break;
		}
	}

	lckGlobalPlan.unlock();

	if (globalPlannerParams.debug == 1)
		std::cout << "Global Planner : Finished finding current edge "
				<< std::endl;

}

void GlobalPlanner::computeGlobalPlan(double robotX, double robotY,
		int startType) {
	// Finding shortest path using Dijkstra

	// Clear last route
	nodesToVisit.clear();
	// We also invalidate the previously found route
	clearRouteInGlobalPlan();

	// Is it the same edge ?
	if (startType == 2 && goalType == 2) {
		int startMin = startId[0] < startId[1] ? startId[0] : startId[1];
		int startMax = startId[0] + startId[1] - startMin;
		int goalMin = goalId[0] < goalId[1] ? goalId[0] : goalId[1];
		int goalMax = goalId[0] + goalId[1] - goalMin;

		// The same edge
		if (goalMin == startMin && goalMax == startMax) {
			// Skip dijkstra - go directly to target
			if (globalPlannerParams.debug == 1) {
				std::cout << "Global Planner : Goal straight ahead to target"
						<< std::endl;
				return;
			}
		}
	}
	// Is it the same node ?
	if (startType < 2 && goalType < 2) {
		// The same node
		if (goalId[goalType] == startId[startType]) {
			// Skip dijkstra - go directly to target
			if (globalPlannerParams.debug == 1) {
				std::cout << "Global Planner : Goal straight ahead to target"
						<< std::endl;
				return;
			}
		}
	}

	// Containers for Dijkstra
	std::vector<double> distance(edges.size(), -1.0);
	std::vector<int> previous(edges.size(), -1.0);
	std::priority_queue<std::pair<double, int>,
			std::vector<std::pair<double, int>>, myPQueueComparison> pqueue;

	// Starting values from single node or edge
	// We start from node 0
	if (startType == 0) {
		distance[startId[0]] = startIdDist[0];
		previous[startId[0]] = -2.0;
		pqueue.push(std::make_pair(startIdDist[0], startId[0]));
	}
	// We start from node 1
	else if (startType == 1) {
		distance[startId[1]] = startIdDist[1];
		previous[startId[1]] = -2.0;
		pqueue.push(std::make_pair(startIdDist[1], startId[1]));
	}
	// We start from edge 0-1
	else if (startType == 2) {
		for (int i = 0; i < 2; i++) {
			distance[startId[i]] = startIdDist[i];
			previous[startId[i]] = -2.0;
			pqueue.push(std::make_pair(startIdDist[i], startId[i]));
		}
	}

	cout << "PQUEUE SIZE : " << pqueue.size() << " startType == " << startType
			<< " goalType == " << goalType<< endl;
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
			if (fabs(distance[*listIter] + 1) < 0.0000001
					|| distance[*listIter]
							> distance[idToProcess] + additionalDistance) {
				// Shorter path has been found
				distance[*listIter] = distance[idToProcess]
						+ additionalDistance;
				previous[*listIter] = idToProcess;

				// We add as a new node to process
				pqueue.push(std::make_pair(distance[*listIter], *listIter));
			}
		}

	}

	// Conclusions
	if (globalPlannerParams.debug == 1) {
		std::cout << "Global Planner : Printing ids from backward to get to id="
				<< goalId[0] << " or " << goalId[1] << " from ids="
				<< startId[0] << " or " << startId[1] << std::endl;

		std::cout << std::endl << " GOAL DISTANCE : " << distance[goalId[0]]
				<< " or " << distance[goalId[1]] << std::endl;
	}

	// Drastic change of plans - wait a little bit
	startTime = std::chrono::high_resolution_clock::now();
	double currentPlanDistance = std::min(distance[goalId[0]],distance[goalId[1]]);
	if ( goalType < 2 )
	{
		currentPlanDistance = distance[goalId[ goalType]];
	}


	// Plan different by over some meters
	std::chrono::milliseconds tmpTime = std::chrono::duration_cast
					< std::chrono::milliseconds
			> (std::chrono::high_resolution_clock::now()
					- changePlanDelayStartTime);
	if (fabs(previousPlanDistance+1) > 0.0001 && fabs(currentPlanDistance - previousPlanDistance)
			> globalPlannerParams.changedPlanThreshold
			&& tmpTime.count() > globalPlannerParams.changedPlanDelayTime && !weShouldWait)
	{
		weShouldWait = true;
		localPlanner->stopLocalPlanner();
		waitingStartTime = std::chrono::high_resolution_clock::now();
	}
	else if (weShouldWait == true)
	{
		std::chrono::milliseconds time = std::chrono::duration_cast
					< std::chrono::milliseconds
					> (std::chrono::high_resolution_clock::now() - waitingStartTime);
		if (time.count() > globalPlannerParams.changedPlanWaitingTime)
		{
			weShouldWait = false;
			localPlanner->startLocalPlanner();
			changePlanDelayStartTime = std::chrono::high_resolution_clock::now();
		}
	}
	previousPlanDistance = currentPlanDistance;


	// Let's check which node or the node of the final edge is closer to us
	int finalGoalId = findGoalNodeId(finalGoalId, distance);

	if (globalPlannerParams.debug == 1) {
			std::cout << "Global Planner : FinalGoalid =" << finalGoalId << std::endl;
			std::cout << "Global Planner : startType =" << startType << std::endl;
	}
	nodesToVisit.push_back(finalGoalId);

	// Print the route until we are in our position
	int i = finalGoalId;
	while (true) {

		if (globalPlannerParams.debug == 1)
			std::cout << "Global Planner : " << i << " " << previous[i]<<std::endl;


		if ((startType == 0 || startType == 2) && i == startId[0]) {
			break;
		} else if ((startType == 1 || startType == 2) && i == startId[1]) {
			break;
		}

		int k = previous[i];

		// We go using this edge
		Edge e;
		e.x1 = nodePosition[i].first, e.y1 = nodePosition[i].second;
		e.x2 = nodePosition[k].first, e.y2 = nodePosition[k].second;

		std::unique_lock < std::mutex > lckGlobalPlan(mtxGlobalPlan);
		std::set<GlobalPlanner::Edge>::iterator it =
				globalPlanInfo.edges.begin();

		// We look for this edge
		bool found = false;
		for (; it != globalPlanInfo.edges.end(); i++) {
			if (areEdgesEqual(e, *it)) {
				// We mark them as chosen in our route
				GlobalPlanner::Edge tmp = *it;
				it = globalPlanInfo.edges.erase(it);
				tmp.isChosen = true;
				globalPlanInfo.edges.insert(tmp);
				found = true;
			} else
				++it;
		}

		lckGlobalPlan.unlock();

		if (globalPlannerParams.debug == 1) {
			if (found == false)
				std::cout
						<< "Global Planner: NIE ZNALEZIONO KRAWEDZI z drogi w grafie "
						<< std::endl;
		}
		// Let's move using the route
		i = k;
		nodesToVisit.push_front(i);
	}

	std::cout << "Global Planner : Found a path to follow" << std::endl;
}

void GlobalPlanner::goDirectlyToTarget(double robotX, double robotY,
		bool& recomputePlan) {

	// One time
	if (planningStage != preciselyToGoal && planningStage != preciselyToStart) {

		if (globalPlannerParams.debug == 1)
			printf("Global planner: We are close to goal - precise navigation\n");

		// We change to precise navigation
		planningStage = static_cast<PlanningStage>(((int) (planningStage) + 1)
				% 4);
		startTime = std::chrono::high_resolution_clock::now();
		localPlanner->setPreciseSpeed();
	}

	// We update the direction to the goal
	double x = goalX - robotX;
	double y = goalY - robotY;
	setGoalDirection(atan2(y, x));


	// Next time
	std::chrono::milliseconds time = std::chrono::duration_cast
			< std::chrono::milliseconds
			> (std::chrono::high_resolution_clock::now() - startTime);
	if (time.count() > globalPlannerParams.preciseToGoalMaxTime) {

		if (globalPlannerParams.debug == 1) {
			printf(
				"Global planner: We precisely reached the goal - let's change the plan\n");
		}

		// Going back to normal operation
		planningStage = static_cast<PlanningStage>(((int) (planningStage) + 1)
				% 4);

		// stopLocalPlanner
		localPlanner->stopLocalPlanner();

		// Let's beep !!!:D
		if (globalPlannerParams.sound == 1) {
			int tmp = system("espeak -v en-us+1 -s 150 \"Target reached. I'm going home.\"");
		}

		// Going back home :)
		if (planningStage == toStart) {
			goalX = 0.0;
			goalY = 0.0;
			updateGoal();
			recomputePlan = true;
			// startLocalPlanner
			localPlanner->setNormalSpeed();
			localPlanner->startLocalPlanner();
		} else if (planningStage == toGoal) {
			 if (globalPlannerParams.debug == 1) {
				 printf("Global planner: robot finished it's job\n");
			 }
			 if (globalPlannerParams.sound == 1) {
				 int tmp = system("espeak -v en-us+1 -s 150 \"Finished my job\"");
			 }
		} else if (globalPlannerParams.debug == 1) {
				printf("Global planner: Error in global planner state\n");
		}
	}
}

void GlobalPlanner::chooseNextSubGoal(double robotX, double robotY,
		bool &recomputePlan) {
	if (globalPlannerParams.debug == 1) {
		std::cout<<"Global planner : choosing subgoals : nodesToVisit = " << nodesToVisit.size() << std::endl;
	}

	// There is only goal ahead -> let's precisely navigate towards it
	if (nodesToVisit.size() == 0) {

		if (globalPlannerParams.debug == 1) {
			std::cout << "Global Planner : we are heading directly to target"
					<< std::endl;
			usleep(1000 * 4000);
		}

		// Let's go directly to target
		goDirectlyToTarget(robotX, robotY, recomputePlan);
	}
	// There are some nodes to visit before reaching the target
	else {
		// We check the global plan
		bool foundSubgoal = false;

		double originalGoalTheta = goalTheta;
		for (std::list<int>::iterator it = nodesToVisit.begin();;) {
			std::pair<double, double> tmp = nodePosition[*it];
			double dist = pow(tmp.first - robotX, 2)
					+ pow(tmp.second - robotY, 2);

			double x = tmp.first - robotX;
			double y = tmp.second - robotY;
			double direction = atan2(y,x);
			double directionDifference = fabs(direction - originalGoalTheta)*180.0/M_PI;

			// This is the node we go to (because it is further away than threshold or the direction is the same):
			if (sqrt(dist) > globalPlannerParams.subgoalThreshold && foundSubgoal == false) {

				setGoalDirection(direction);

				if (globalPlannerParams.debug == 1) {
					std::cout << "Global Planner : node : " << tmp.first << " "
							<< tmp.second << std::endl;
					std::cout << "Global Planner : robot position : " << robotX
							<< " " << robotY << std::endl;
					std::cout << "Global Planner : distance to subgoal : " << x
							<< " " << y << " : Dist = " << sqrt(x * x + y * y)
							<< std::endl;
				}

				foundSubgoal = true;
			} else if (sqrt(dist) > globalPlannerParams.subgoalThreshold
					&& sqrt(dist) < globalPlannerParams.subgoalThreshold2
					&& directionDifference
							< globalPlannerParams.subgoalAngularThreshold) {

				setGoalDirection(direction);

				std::list<int>::iterator it2 = nodesToVisit.begin();
				while(it2!=it)
				{
					nodesToVisit.pop_front();
					it2 = nodesToVisit.begin();
				}

				if (globalPlannerParams.debug == 1) {
					std::cout << "Global Planner : angular choice - node : " << tmp.first << " "
							<< tmp.second << std::endl;
					std::cout << "Global Planner : angular choice - robot position : " << robotX
							<< " " << robotY << std::endl;
					std::cout << "Global Planner : angular choice - distance to subgoal : " << x
							<< " " << y << " : Dist = " << sqrt(x * x + y * y)
							<< std::endl;
				}

			}
			else {
				if (foundSubgoal)
					break;

				if (globalPlannerParams.debug == 1) {
					std::cout << "Global Planner : removing node" << std::endl;
				}

				nodesToVisit.pop_front();
				it = nodesToVisit.begin();
			}
		}
		// We can go directly to target
		if (foundSubgoal == false) {
			if (globalPlannerParams.debug == 1)
			{
				std::cout << "Global Planner : we are heading directly to target" << std::endl;
				usleep(1000*4000);
			}
			goDirectlyToTarget(robotX, robotY, recomputePlan);
		}
	}
	if (globalPlannerParams.debug == 1)
		std::cout << "Global Planner : We are heading with theta = " << goalTheta
				<< " (in degrees)" << std::endl;
}

// Helping methods

void GlobalPlanner::clearRouteInGlobalPlan() {
	std::unique_lock < std::mutex > lckGlobalPlan(mtxGlobalPlan);
	std::set<GlobalPlanner::Edge>::iterator it = globalPlanInfo.edges.begin();

	for (; it != globalPlanInfo.edges.end();) {
		if (it->isChosen == true) {
			GlobalPlanner::Edge tmp = *it;
			it = globalPlanInfo.edges.erase(it);
			tmp.isChosen = false;
			globalPlanInfo.edges.insert(tmp);
		} else
			++it;
	}
	lckGlobalPlan.unlock();
}

void GlobalPlanner::findClosestEdge(double X, double Y, int &id1, int &id2,
		double &minDistance, int &distanceType) {

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

			int distType = 2;
			// The distance is computed to the 1st vertex
			if (ab <= 0 && cb >= 0) {
				d = sqrt((x1 - X) * (x1 - X) + (y1 - Y) * (y1 - Y));
				distType = 0;
			}
			// ... and to the second vertex
			else if (ab >= 0 && cb <= 0) {
				d = sqrt((x2 - X) * (x2 - X) + (y2 - Y) * (y2 - Y));
				distType = 1;
			}

			// Let's see if this edge is the closest one to the currently check position
			if (d < minDistance || minDistance < 0) {
				minDistance = d;
				id1 = i;
				id2 = *lIt;
				distanceType = distType;
			}
		}
	}
	if (globalPlannerParams.debug == 1)
		std::cout << "Global Planner: the ids of the closest edge are : " << id1
				<< " and " << id2 << " and the distance is " << minDistance
				<< " meters and distanceType is " << distanceType << std::endl;
}

bool GlobalPlanner::areEdgesEqual(Edge e, Edge f) {
	checkAndCorrectEdgeConvention(e);
	checkAndCorrectEdgeConvention(f);

	double d = fabs(e.x1 - f.x1) + fabs(e.y1 - f.y1) + fabs(e.x2 - f.x2)
			+ fabs(e.y2 - f.y2);
	if (d > 0.000001)
		return false;
	return true;
}

void GlobalPlanner::switchEdge(Edge &e) {
	double a = e.x1;
	e.x1 = e.x2;
	e.x2 = a;

	a = e.y1;
	e.y1 = e.y2;
	e.y2 = a;
}

void GlobalPlanner::checkAndCorrectEdgeConvention(Edge &e) {
	if (e.x1 < e.x2)
		switchEdge(e);
	else if (fabs(e.x1 - e.x2) < 0.000001 && e.y1 < e.y2)
		switchEdge(e);
}

double GlobalPlanner::computeDistance(std::pair<double, double> a,
		std::pair<double, double> b) {
	return std::sqrt(
			(a.first - b.first) * (a.first - b.first)
					+ (a.second - b.second) * (a.second - b.second));
}

int GlobalPlanner::findGoalNodeId(int finalGoalId,
		std::vector<double> distance) {
	if (goalType == 0)
		finalGoalId = goalId[0];
	else if (goalType == 1)
		finalGoalId = goalId[1];
	else {
		if (distance[goalId[0]]
				+ computeDistance(nodePosition[goalId[0]],
						std::make_pair(goalX, goalY))
				< distance[goalId[1]]
						+ computeDistance(nodePosition[goalId[1]],
								std::make_pair(goalX, goalY)))
			finalGoalId = goalId[0];
		else
			finalGoalId = goalId[1];
	}

	return finalGoalId;
}

void GlobalPlanner::setGoalDirection(double theta) {
	std::unique_lock < std::mutex > lckGoalTheta(mtxGoalTheta);
	goalTheta = theta * 180.0 / 3.14159265;
	lckGoalTheta.unlock();
}

void GlobalPlanner::setGoalDirectionInDegrees(double theta) {
	std::unique_lock < std::mutex > lckGoalTheta(mtxGoalTheta);
	goalTheta = theta;
	lckGoalTheta.unlock();
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

		robotDrive1->setMotorSpeed(
				max(min((int) (motLeft * ROBOT_DRIVE_MAX / 100),
				ROBOT_DRIVE_MAX), -ROBOT_DRIVE_MAX));
		robotDrive2->setMotorSpeed(
				max(min((int) (-motRight * ROBOT_DRIVE_MAX / 100),
				ROBOT_DRIVE_MAX), -ROBOT_DRIVE_MAX));
	}
	lck.unlock();
}

//----------------------ACCESS TO COMPUTED DATA
GlobalPlanner::GlobalPlanInfo GlobalPlanner::getGlobalPlan() {
	std::unique_lock < std::mutex > lckGlobalPlan(mtxGlobalPlan);
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

	std::unique_lock < std::mutex > lckGoalTheta(mtxGoalTheta);
	float returnValue = goalTheta;
	lckGoalTheta.unlock();

	return goalTheta;
}


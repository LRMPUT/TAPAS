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

class myPQueueComparison {
public:
	myPQueueComparison() {
	}
	bool operator()(const std::pair<double, int>& lhs,
			const std::pair<double, int>&rhs) const {
		return (lhs.first < rhs.second);
	}
};

GlobalPlanner::GlobalPlanner(Robot* irobot, TiXmlElement* settings) :
		robot(irobot), robotDrive1(NULL), robotDrive2(NULL), startOperate(false) {
	cout << "GlobalPlanner()" << endl;



	readSettings(settings);
	globalPlannerThread = std::thread(&GlobalPlanner::run, this);

	localPlanner = new LocalPlanner(robot, this, settings);

	cout << "End GlobalPlanner()" << endl;
}

GlobalPlanner::~GlobalPlanner() {
	cout << "~GlobalPlanner()" << endl;
	stopThread();
	closeRobotsDrive();
	cout << "End ~GlobalPlanner()" << endl;
}

void GlobalPlanner::readSettings(TiXmlElement* settings)
{
	TiXmlElement* pGlobalPlanner;
	pGlobalPlanner = settings->FirstChildElement("GlobalPlanner");

	if (pGlobalPlanner->QueryIntAttribute("runThread", &globalPlannerParams.runThread)
			!= TIXML_SUCCESS) {
		throw "Bad settings file - wrong value for globalPlanner Thread";
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
	printf("GlobalPlanner -- runhomologation: %d\n", globalPlannerParams.runHomologation);
	printf("GlobalPlanner -- mapFile: %s\n", globalPlannerParams.mapFile.c_str());
	printf("GlobalPlanner -- goal latitude: %f \n", globalPlannerParams.latitude);
	printf("GlobalPlanner -- goal longitude: %f \n", globalPlannerParams.longitude);
}


void GlobalPlanner::run() {

	if (globalPlannerParams.runThread) {

		std::cout<<"GPS: Waiting for GPS set zero" <<std::endl;
		while (!robot->isGpsOpen()
				|| robot->isSetZero() == false)
			usleep(200);


		std::cout<<"GPS: Waiting on Fix status" <<std::endl;
		while (robot->gpsGetFixStatus() == 1 && globalPlannerParams.runThread) {
			usleep(200);
		};
		std::cout<<"GPS: Stopped waiting on fix status" <<std::endl;

		const char *cstr = globalPlannerParams.mapFile.c_str();
		readOpenStreetMap(cstr);

		double lon = GPS::decimal2Nmea(globalPlannerParams.longitude);
		double lat = GPS::decimal2Nmea(globalPlannerParams.latitude);
		setGoal(robot->getPosX(lon)/1000,
				robot->getPosY(lat)/1000);

		//localPlanner->startLocalPlanner();
	}
	while (globalPlannerParams.runThread) {

//		if (startOperate) {

			updateRobotPosition();
			findClosestStartingEdge(robotX, robotY);




//			computeGlobalPlan();
//			chooseNextSubGoal();
//			updateHeadingGoal();

//		}
		std::chrono::milliseconds duration(150);
		std::this_thread::sleep_for(duration);
	}
	if (globalPlannerParams.runThread) {
//		localPlanner->stopLocalPlanner();
	}
}

void GlobalPlanner::computeGlobalPlan() {
	// Finding shortest path using Dijkstra

	// Containers for Dijkstra
	std::vector<double> distance(edges.size(), -1.0);
	std::vector<int> previous(edges.size(), -1.0);
	std::priority_queue<std::pair<double, int>,
			std::vector<std::pair<double, int>>, myPQueueComparison> pqueue;

	// Going through nodes
	distance[ startingNodeIndex[0] ] = 0;
	distance[ startingNodeIndex[1] ] = 0;

	previous[ startingNodeIndex[0] ] = -2.0;
	previous[ startingNodeIndex[1] ] = -2.0;

	pqueue.push(std::make_pair(startingNodeDist[0],  startingNodeIndex[0]));
	pqueue.push(std::make_pair(startingNodeDist[1],  startingNodeIndex[1]));

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
				previous[*listIter] = idToProcess;
				pqueue.push(std::make_pair(distance[*listIter], *listIter));
			}
		}

	}

//	// Conclusions
//	std::cout << "Ended computation from nodes : " << closestEdge.first << " "
//			<< closestEdge.second << std::endl;
//	int ile = 0, ogolem = 0;
//	for (int i = 0; i < edges.size(); i++) {
//		if (edges[i].size() != 0) {
//			ogolem++;
//			std::cout << "Distance to " << i << " is equal to " << distance[i]
//					<< std::endl;
//			if (distance[i] > 0)
//				ile++;
//		}
//	}
//	std::cout << "Possiblity to access " << ile * 100.0 / ogolem
//			<< "% of all nodes" << std::endl;
//

	std::cout << "Printing ids from backward to get to id=" << goalId
			<< " from ids=" << startingNodeIndex[0] << " or "
			<< startingNodeIndex[1] << std::endl;
	int i = goalId;
	while( true )
	{
		int k = previous[i];
		std::cout<<"From "<<i<<" to " << k << std::endl;
		i = k;

		Edge edge;
		edge.isChosen = true;
		edge.x1 = nodePosition[i].first;
		edge.y1 = nodePosition[i].second;
		edge.x2 = nodePosition[k].first;
		edge.y2 = nodePosition[k].second;
		globalPlanInfo.edges.insert(edge);

		if(distance[i] == -2)
		{
			break;
		}
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
	startOperate = false;
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

	std::cout << "Global planner - robot position: " << robotX << " " <<robotY<<std::endl;
}

void GlobalPlanner::readOpenStreetMap(const char *mapName) {
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
//			printf("Id lat lon: %s %s %s\n", pId, pLat, pLon);
//			printf("lat lon: %f %f\n", atof(pLat), atof(pLon));
//			printf("X Y : %f %f\n", robot->getPosX(atof(pLat)*100), robot->getPosY(atof(pLon)*100));
			char *end;
			idConversion.insert(std::make_pair(strtol(pId, &end, 10), i));


			double lon = GPS::decimal2Nmea(atof(pLon)*100);
			double lat = GPS::decimal2Nmea(atof(pLat)*100);


			nodePosition.push_back(
					std::make_pair(robot->getPosX(lat)/1000,
							robot->getPosY(lon)/1000));

			printf("Edge: %f %f -- %f %f (in meters)\n", lat, lon, robot->getPosX(lat)/1000, robot->getPosY(lon)/1000);
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

	std::cout << "Ways found counter : " << i << std::endl;
	std::cout << "Edges counter : " << edges.size() << std::endl;

	std::unique_lock<std::mutex> lckGlobalPlan(mtxGlobalPlan);
	bool start = true;
	double minX, maxX, minY, maxY;

	for (int i = 0; i < edges.size(); i++) {
		for (list<int>::iterator lIt = edges[i].begin(); lIt != edges[i].end();
				++lIt) {
			std::pair<double, double> node1 = nodePosition[i], node2 =
					nodePosition[*lIt];

			Edge edge;
			edge.isChosen = false;

			if ((node1.first < node2.first)
					|| (node1.first == node2.first && node2.second < node2.second ))
			{
				edge.x1 = node1.first;
				edge.y1 = node1.second;
				edge.x2 = node2.first;
				edge.y2 = node2.second;
			}
			else
			{
				edge.x1 = node2.first;
				edge.y1 = node2.second;
				edge.x2 = node1.first;
				edge.y2 = node1.second;
			}

			globalPlanInfo.edges.insert(edge);

			if (start)
			{
				minX = edge.x1 > edge.x2 ? edge.x2 : edge.x1 ;
				maxX = edge.x1 > edge.x2 ? edge.x1 : edge.x2 ;
				minY = edge.y1 > edge.y2 ? edge.y2 : edge.y1 ;
				maxY = edge.y1 > edge.y2 ? edge.y1 : edge.y2 ;
				start = false;
			}
			else
			{
				if (edge.x1 > maxX)
					maxX = edge.x1;
				if (edge.x1 < minX)
					minX = edge.x1;

				if (edge.y1 > maxY)
					maxY = edge.y1;
				if (edge.y1 < minY)
					minY = edge.y1;

				if (edge.x2 > maxX)
					maxX = edge.x2;
				if (edge.x2 < minX)
					minX = edge.x2;

				if (edge.y2 > maxY)
					maxY = edge.y2;
				if (edge.y2 < minY)
					minY = edge.y2;

			}
		}

	}
	lckGlobalPlan.unlock();


//	for (int i = 0; i < nodePosition.size();i++)
//	{
//		if ( start )
//		{
//			minX = maxX = nodePosition[i].first;
//			minY = maxY = nodePosition[i].second;
//			start = false;
//		}
//		else
//		{
//			if ( nodePosition[i].first > maxX)
//				maxX = nodePosition[i].first;
//			if ( nodePosition[i].first < minX)
//				minX = nodePosition[i].first;
//
//			if ( nodePosition[i].second > maxY)
//				maxY = nodePosition[i].second;
//			if ( nodePosition[i].second < minY)
//				minY = nodePosition[i].second;
//		}
//
//	}
	std::cout << "Min/Max : " << minX << " " << maxX << " " << minY << " "
				<< maxY << std::endl;

	std::cout << "robotX : " << robotX << "\trobotY" << robotY <<endl;
	if ( robotX > maxX)
		maxX = robotX;
	else if (robotX < minX)
		minX = robotX;

	if (robotY > maxY)
		maxY = robotY;
	else if (robotY < minY)
		minY = robotY;

	std::unique_lock<std::mutex> lckGlobalPlan2(mtxGlobalPlan);

//	robotX = 0.0;
//	robotY = 0.0;

	globalPlanInfo.robotX = robotX;
	globalPlanInfo.robotY = robotY;

	globalPlanInfo.minX = minX;
	globalPlanInfo.maxX = maxX;
	globalPlanInfo.minY = minY;
	globalPlanInfo.maxY = maxY;

	std::cout << "Min/Max : " << minX << " " << maxX << " " << minY << " "
			<< maxY << std::endl;

	std::cout<<"Edges size: " << globalPlanInfo.edges.size()<<std::endl;

	lckGlobalPlan2.unlock();
	std::cout<<"Global Planner: end readosmMap"<<std::endl;
}

void GlobalPlanner::findClosestStartingEdge(double X, double Y) {

	double minDist = -1;
	int bestVersion;
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

			// According to :
			// dev.cdur.pl/Artykuly/Odleglosc-punktu-od-odcinka
			double x1 = node1.first, y1 = node1.second;
			double x2 = node2.first, y2 = node2.second;

//			double u_nom = (x2 - x1) * (X - x1) + (y2 - y1) * (Y - y1);
//			double u_den = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
//			double u = u_nom / u_den;
//			double x3 = x1 + u * (x2 - x1);
//			double y3 = y1 + u * (y2 - y1);

			double a_x = x1 - X, a_y = y1 - Y;
			double b_x = x1 - x2, b_y = y1 - y2;
			double c_x = x2 - X, c_y = y2 - Y;

			double ab = a_x * b_x + a_y * b_y;
			double cb = -c_x * b_x - c_y * b_y;

			int version = 0;// ab > 0 && cb > 0
			if (ab <= 0 && cb >= 0)
			{
				version = 1;
				d = sqrt( (x1 - X)*(x1 - X) + (y1 - Y)*(y1 - Y) );
			}
			else if (ab >= 0 && cb <= 0)
			{
				version = 2;
				d = sqrt( (x2 - X)*(x2 - X) + (y2 - Y)*(y2 - Y) );
			}
//			else if (ab < 0 && cb < 0)
//			{
//				printf("Values: %f %f %f %f %f %f\n", x1, y1, x2, y2, X, Y);
//				printf("Oh shit! It is wrong! dist = %f\n", u);
//				cin >>d;
//			}

			if (d < minDist || minDist < 0) {
				minDist = d;
				bestVersion = version;
				save1 = i;
				save2 = *lIt;
			}
		}
	}
	std::cout << "MinDist : " << minDist << " Version: " << bestVersion << std::endl;

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

	startingNodeIndex[0] = save1;
	startingNodeIndex[1] = save2;
	startingNodeDist[0] = distance1;
	startingNodeDist[1] = distance2;

	std::unique_lock < std::mutex > lckGlobalPlan(mtxGlobalPlan);
	std::cout << "Managed to get through mutex " << std::endl;

	if ((nodePosition[save1].first > nodePosition[save2].first)
			|| (nodePosition[save1].first == nodePosition[save2].first
					&& nodePosition[save1].second < nodePosition[save2].second))
	{
		int tmp = save2;
		save2 = save1;
		save1 = tmp;
	}

	double x1 = nodePosition[save1].first, y1 = nodePosition[save1].second;
	double x2 = nodePosition[save2].first, y2 = nodePosition[save2].second;
	std::set<GlobalPlanner::Edge>::iterator it = globalPlanInfo.edges.begin();
	int i=0;
	for (; it != globalPlanInfo.edges.end(); i++) {
		if (fabs(it->x1 - x1) < 0.0001
				&& fabs(it->y1 - y1) < 0.0001
				&& fabs(it->x2 - x2) < 0.0001
				&& fabs(it->y2 - y2) < 0.0001) {
			globalPlanInfo.curEdge = i;
			std::cout << "Global planner current edge: " << i << " | " << it->x1
					<< " " << it->y1 << " " << it->x2 << " " << it->y2
					<< std::endl;
		}
		if ( it->isChosen == true )
		{
			GlobalPlanner::Edge tmp = *it;
			globalPlanInfo.edges.erase(it++);
			tmp.isChosen = false;
			globalPlanInfo.edges.insert(tmp);
		}
		else
			++it;
	}

	lckGlobalPlan.unlock();

	std::cout << "Finished finding current edge "<< std::endl;
}

void GlobalPlanner::setGoal(double X, double Y) {
	goalX = X;
	goalY = Y;
	double bestDistance = -1;
	int bestId = -1;
	for (int i=0;i<nodePosition.size();i++)
	{
		std::pair<double, double> tmp = nodePosition[i];
		double dist = (tmp.first - X) * (tmp.first - X) + (tmp.second - X) * (tmp.second - Y);
		if (dist > bestDistance || bestDistance == -1)
		{
			bestDistance = dist;
			bestId = i;
		}
	}
	goalId = bestId;
}

void GlobalPlanner::stopThread() {
	globalPlannerParams.runThread = false;
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
	currentGoal = 90.0;

	return currentGoal;
}


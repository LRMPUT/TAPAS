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

LocalPlanner::LocalPlanner(Robot* irobot, GlobalPlanner* planer) :
		robot(irobot),
		globalPlanner(planer),
		startOperate(false)
{
	runThread = true;
	localPlannerThread = std::thread(&LocalPlanner::run, this);
}

LocalPlanner::~LocalPlanner(){
	cout << "~LocalPlanner()" << endl;
	stopThread();
	cout << "End ~LocalPlanner()" << endl;
}

void LocalPlanner::run(){
	while(runThread){
		if(startOperate){
			executeVFH();
		}
		std::chrono::milliseconds duration(50);
		std::this_thread::sleep_for(duration);
	}
}

void LocalPlanner::executeVFH(){


	updateHistogram();
	findFreeSectors();
	// smoothHistogram();
	setGoalDirection();
	/// goal direction in the local map coordiantes
	determineGoalInLocalMap();
	/// optimal direction in the local map - nearest to the goal
	findOptimSector();
	determineDriversCommand();

}

float LocalPlanner::getLocalDirection(){
	return bestDirection;
}

void LocalPlanner::setGoalDirection(){

	///// this part of code is only for  test purposes
	// to test the straight ahead direction of movement:
	// logic flow: get current orientation ane keep it
	static bool setStraighAheadDirection = false;
	static float direction = 0;

	if (!setStraighAheadDirection){
	Mat posLocalToGlobalMap = robot->getLocalMapPosInGlobalMap();
	/// and convert this orentation to euler yaw
	direction = RotMatToEulerYaw(posLocalToGlobalMap);
	direction = 180/PI*direction;
	setStraighAheadDirection = true;
	}
	///////////////



	//goalDirection = globalPlanner->getHeadingToGoal();
	goalDirection = direction; // => test go straight ahead
}


void LocalPlanner::initHistogram(){

	for (int sect = 0; sect < HIST_SECTORS; sect++){
		histSectors.push_back(0.0);
	}
}

void LocalPlanner::localPlanerTest(){

	initHistogram();
	while (true){
		executeVFH();
	}
}

float LocalPlanner::RotMatToEulerYaw(Mat rotMat){

	float R11 = rotMat.at<float>(1, 1);
	float R21 = rotMat.at<float>(2, 1);
	float R31 = rotMat.at<float>(3, 1);

	float teta1 = -asin(R31);
	float teta2 = PI - teta1;
	float fi1 = 0; float fi2 = 0;

	if (R31 != 1 && R31 != -1){

		fi1 = atan2(R21/cos(teta1), R11/cos(teta1));
		fi2 = atan2(R21/cos(teta2), R11/cos(teta2));
	}
	else
	{
		teta1 = 0;
		teta2 = 0;
	}

}


void LocalPlanner::updateHistogram(){

	float angPosition;
	float density;

	const float maxDistance = 1.41*(float)MAP_SIZE;
	const float hist_A = 1;
	const float hist_B =  hist_A/maxDistance;
	/// Get current Constraints Raster Map
	Mat constraints = robot->getMovementConstraints();
	Mat posImuMapCenter = robot->getPosImuConstraintsMapCenter();

	// update position of the robot
	float addX = posImuMapCenter.at<float>(0, 3)/MAP_RASTER_SIZE;
	float addY = posImuMapCenter.at<float>(1, 3)/MAP_RASTER_SIZE;

	int curRobCellX = MAP_SIZE/2 + addX;
	int curRobCellY = MAP_SIZE/2 + addY;
	int sector = 0;
	cout<<"RobotX"<<curRobCellX<<endl;
	cout<<"RobotY"<<curRobCellY<<endl;

	/// for each cell of the raster map
	for(int x = MAP_SIZE/2; x < MAP_SIZE ; x++){
		for(int y = MAP_SIZE; y > 0; y--){
			///angular position
			angPosition = 180/PI*atan2(float(x - curRobCellX), y - curRobCellY);
			cout<<"angPosition"<<angPosition<<endl;
			/// value of obstacle vector
			density = pow(constraints.at<float>(x, y),2)*
					(hist_A - hist_B*sqrt(pow((float)(x - curRobCellX),2)
					+ pow((float)(y - curRobCellY), 2)));

			/// update proper sector of histogram
			histSectors.at(floor((angPosition/10))) += density;
		}
	}

	for (int sect = 0; sect < HIST_SECTORS; sect++){
		std::cout<<histSectors.at(sect)<<endl;
	}
}

void LocalPlanner::smoothHistogram(){

	int winWidthLeft = 0;
	int winWidthRight = 0;
	float sumOfDensity = 0.0;

	for (int sec = 0; sec < HIST_SECTORS; sec++){

		/// set default width
		winWidthLeft = 3;
		winWidthRight = 3;
		/// omit the window boundaries
		for (sec = 3; sec <= HIST_SECTORS-3; sec++){
			/// clear sum of density
			sumOfDensity = 0.0;

			for (int i = sec-3; i < sec+3; i++){
				sumOfDensity += histSectors.at(i);
			}

			histSectors.at(sec) = sumOfDensity/7;
		}
	}
}

void LocalPlanner::findFreeSectors(){

	freeSectors.clear();

	bool begOfFreeSec = false;

	for (int sec = 0; sec < HIST_SECTORS; sec++){

		if (histSectors.at(sec) == 0 && !begOfFreeSec){
			freeSectors.push_back(sec);
			begOfFreeSec = true;
		}

		if (histSectors.at(sec) != 0 && begOfFreeSec){
					freeSectors.push_back(sec);
					begOfFreeSec = false;
				}

	}
	cout<<"Wolne sektory:"<<endl;
	for (int i = 0; i < freeSectors.size(); i++){
	cout<<freeSectors.at(i)<<endl;
	}
}

/// return Goal Direction converted from Global Map to
/// local MAP
void LocalPlanner::determineGoalInLocalMap(){

	float globalYaw = 0.0;
	float localToGlobalDifference = 0.0;

	cout<<"determineGoalInLocalMap"<<endl;


	/// get Orientation of Local Map in Global Map
	Mat posLocalToGlobalMap = robot->getLocalMapPosInGlobalMap();
	/// and convert this orentation to euler yaw
	globalYaw = RotMatToEulerYaw(posLocalToGlobalMap);
	globalYaw = 180/PI*globalYaw;
	cout<<"Global_YAW: "<<globalYaw<<endl;
	// calculate difference between local and global map in orientation
	localToGlobalDifference = goalDirection - globalYaw;
	cout<<"Difference: "<<localToGlobalDifference<<endl;
	goalDirection = goalDirection - globalYaw;

	cout<<"determineGoalInLocalMap"<<endl;
}



void LocalPlanner::findOptimSector(){

	cout<<"calculateLocalDirection START"<<endl;
	// goal direction+offset/ sectors alpha
	int goalSector = (goalDirection+90)/10;
	cout<<goalSector<<endl;
	int foundSector = -1;
	int selectedSector = 0;



	for (int i = 0; i < freeSectors.size()-1; i = i+2){
		if (goalSector >= freeSectors.at(i) && goalSector <= freeSectors.at(i+1)){
			/// go through the middle of the set of free sectors
			selectedSector = (freeSectors.at(i) + freeSectors.at(i+1))/2;
			foundSector =  selectedSector;
		}
	}
	if (foundSector == -1){
		// set first sector as candidate and look for the better one
		foundSector = 0;

		for (int i = 0; i < freeSectors.size()-1; i++){
			if (pow(float(goalSector-freeSectors.at(i)), 2) < pow(float(goalSector-foundSector), 2)){
				foundSector = freeSectors.at(i);
			}
		}
	}
	// 10 is HIST/ALPHA/2
	bestDirection = foundSector*10;
	cout<<"FoundSector: "<<foundSector<<endl;
	cout<<"calculateLocalDirection END"<<endl;
}


void LocalPlanner::determineDriversCommand(){

	Mat posImuMapCenter = robot->getPosImuConstraintsMapCenter();
	float localYaw = RotMatToEulerYaw(posImuMapCenter);
	localYaw = localYaw*180/PI;

	cout<<"LOCAL_YAW"<<localYaw<<endl;
	cout<<bestDirection<<endl;
	cout<<"Sterowanie"<<endl;

	// if we achieve set point direction then go straight ahead
	// 100 means that we are in target direction with 10 degree margin
	if ((bestDirection+90 - localYaw)*(bestDirection+90 - localYaw) < 100 )
		cout<<"break"<<endl;
		//globalPlanner->setMotorsVel(1000,1000);
	else if (bestDirection+90 > localYaw)
		cout<<"break"<<endl;
		 //globalPlanner->setMotorsVel(1000,-1000);
	else
		cout<<"break"<<endl;
		 //globalPlanner->setMotorsVel(-1000,1000);

	//getchar();
}


void LocalPlanner::stopThread(){
	runThread = false;
	if(localPlannerThread.joinable()){
		localPlannerThread.join();
	}
}




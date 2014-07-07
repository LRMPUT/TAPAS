/*
 * MovementConstaints.cpp
 *
 */

#include "MovementConstraints.h"

#define MAP_SIZE 100	//100x100 MAP_STEPxMAP_STEP cells
#define MAP_STEP 100	//in mm
#define MAP_MARGIN 25	//25 cells margin

using namespace cv;
using namespace std;

MovementConstraints::MovementConstraints(Robot* irobot, TiXmlElement* settings) : robot(irobot) {
	if(!settings){
		throw "Bad settings file - entry MovementConstraints not found";
	}
	readSettings(settings);
	TiXmlElement* pCamera = settings->FirstChildElement("Camera");
	camera = new Camera(this, pCamera);

	runThread = true;
	movementConstraintsThread = std::thread(&MovementConstraints::run, this);

	constraintsMap = Mat(MAP_SIZE, MAP_SIZE, CV_32FC1, Scalar(0));

}

MovementConstraints::~MovementConstraints() {
	delete camera;
}

void MovementConstraints::readSettings(TiXmlElement* settings){
	cameraOrigLaser = readMatrixSettings(settings, "camera_position_laser", 4, 4);
	cameraOrigImu = readMatrixSettings(settings, "imu_position_camera", 4, 4).t();
	groundPlane = readMatrixSettings(settings, "ground_plane_global", 4, 1);
}

cv::Mat MovementConstraints::readMatrixSettings(TiXmlElement* parent, const char* node, int rows, int cols){
	TiXmlElement* ptr = parent->FirstChildElement(node);
	if(!ptr){
		throw (string("Bad settings file - no ") + string(node)).c_str();
	}
	//cout << "node: " << node << ", value: " << ptr->GetText() << endl;
	stringstream tmpStr(ptr->GetText());
	Mat ret = Mat(rows, cols, CV_32FC1);
	for(int row = 0; row < rows; row++){
		for(int col = 0; col < cols; col++){
			float tmpVal;
			tmpStr >> tmpVal;
			ret.at<float>(row, col) = tmpVal;
		}
	}
	return ret;
}

// Main loop of MovementContraints thread.
void MovementConstraints::run(){
	while (runThread) {

		updateConstraintsMap(0, 0, 0);

		//500 ms sleep
		std::chrono::milliseconds duration(500);
		std::this_thread::sleep_for(duration);
	}
}

// Stop MovementConstraints thread.
void MovementConstraints::stopThread(){
	runThread = false;
	movementConstraintsThread.join();
}

void MovementConstraints::updateConstraintsMap(double curGlobX, double curGlobY, double curGlobPhi){
	double curMapX = curGlobX - mapCenterX;
	double curMapY = curGlobY - mapCenterY;
	double curMapPhi = curGlobPhi - mapCenterPhi;

	//przesuwanie mapy
	bool move = false;
	if(curMapX > (MAP_SIZE/2 - MAP_MARGIN) * MAP_STEP){
		move = true;
	}
	else if(curMapX < (-MAP_SIZE/2 + MAP_MARGIN) * MAP_STEP){
		move = true;
	}
	if(curMapY > (MAP_SIZE/2 - MAP_MARGIN) * MAP_STEP){
		move = true;
	}
	else if(curMapY < (-MAP_SIZE/2 + MAP_MARGIN) * MAP_STEP){
		move = true;
	}
	if(move == true){
		mapCenterX = curGlobX;
		mapCenterY = curGlobY;
		mapCenterPhi = curGlobPhi;
	}
	constraintsMap = Scalar(0);

	//polling each constraints module to update map
	insertHokuyoConstraints(constraintsMap, curGlobX - mapCenterX, curGlobY - mapCenterY, curGlobPhi - mapCenterPhi);
}


void MovementConstraints::insertHokuyoConstraints(cv::Mat map, double curMapX, double curMapY, double curMapPhi){
	//Mat hokuyoData = getHokuyoData();

}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
const cv::Mat MovementConstraints::getHokuyoData(){
	return hokuyo.getData();
}

//----------------------ACCESS TO COMPUTED DATA
//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
const cv::Mat MovementConstraints::getMovementConstraints(){
	return Mat(MAP_SIZE, MAP_SIZE, CV_32FC1);
}

//----------------------MENAGMENT OF MovementConstraints DEVICES
//Hokuyo
void MovementConstraints::openHokuyo(std::string port){
	hokuyo.openPort(port);
}

void MovementConstraints::closeHokuyo(){
	hokuyo.closePort();
}

bool MovementConstraints::isHokuyoOpen(){
	return hokuyo.isOpen();
}

//Camera
void MovementConstraints::openCamera(std::vector<std::string> device){
	camera->open(device);
}

void MovementConstraints::closeCamera(){
	camera->close();
}

bool MovementConstraints::isCameraOpen(){
	return camera->isOpen();
}

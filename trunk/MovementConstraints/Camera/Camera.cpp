/*
 * Camera.cpp
 *
 *  Created on: 08-07-2013
 *      Author: jachu
 */

//OpenCV
#include <opencv2/opencv.hpp>
//STL
#include <cmath>
#include <sstream>
//RobotsIntellect
#include "Camera.h"

using namespace boost;
using namespace std;

/*#define CAMERA_Z 1
#define CAMERA_X_ANGLE 45
#define CAMERA_Y_ANGLE 45
#define CAMERAS_COUNT 2
#define ROWS 480
#define COLS 640*/
#define POLY_VERT 4
#define LEFT_CAMERA 0
#define RIGHT_CAMERA 1

using namespace cv;
using namespace std;

Camera::Camera(MovementConstraints* imovementConstraints, TiXmlElement* settings) :
		movementConstraints(imovementConstraints),
		cameraGrid(40)
{
	if(!settings){
		throw "Bad settings file - entry Camera not found";
	}
	readSettings(settings);

	groundPolygons.resize(numCameras);
	for(int cam = 0; cam < numCameras; cam++){
		groundPolygons[cam].resize(numRows/cameraGrid);
		for(int row = 0; row < numRows/cameraGrid; row++){
			groundPolygons[cam][row].resize(numCols/cameraGrid);
			for(int col = 0; col < numCols/cameraGrid; col++){
				groundPolygons[cam][row][col] = new Point[POLY_VERT];
			}
		}
	}
	computeGroundPolygons();
}

Camera::~Camera(){
	for(int cam = 0; cam < numCameras; cam++){
		for(int row = 0; row < numRows/cameraGrid; row++){
			for(int col = 0; col < numCols/cameraGrid; col++){
				delete[] groundPolygons[cam][row][col];
			}
		}
	}
}

void Camera::computeConstraints(std::vector<cv::Mat> image){

}

void Camera::computeGroundPolygons(){
	int rows = numRows/cameraGrid;
	int cols = numCols/cameraGrid;
	for(int im = 0; im < numCameras; im++){
		Mat cornersX(rows + 1, cols + 1, CV_32SC1);
		Mat cornersY(rows + 1, cols + 1, CV_32SC1);
		for(int nrow = 0; nrow < rows; nrow++){
			for(int ncol = 0; ncol < cols; ncol++){
				//computing top left corners
				Point3f point = computePointProjection(	Point2f((float)(ncol - cols/2) / (cols/2),
																-(float)(nrow - rows/2) / (rows/2)),
														im);
				cornersX.at<int>(nrow, ncol) = point.x;
				cornersY.at<int>(nrow, ncol) = point.y;
			}
		}
		for(int ncol = 0; ncol < cols; ncol++){
			//computing bottom left corners
			Point3f point = computePointProjection(	Point2f((float)(ncol - cols/2) / (cols/2),
															-1),
													im);
			cornersX.at<int>(rows, ncol) = point.x;
			cornersY.at<int>(rows, ncol) = point.y;
		}
		for(int nrow = 0; nrow < rows; nrow++){
			//computing top right corners
			Point3f point = computePointProjection(	Point2f(1,
															-(float)(nrow - rows/2) / (rows/2)),
													im);
			cornersX.at<int>(nrow, cols) = point.x;
			cornersY.at<int>(nrow, cols) = point.y;
		}
		//computing bottom right corner
		Point3f point = computePointProjection(	Point2f(1,
														-1),
												im);
		cornersX.at<int>(rows, cols) = point.x;
		cornersY.at<int>(rows, cols) = point.y;
		//Polygons on the ground for each image region
		for(int nrow = 0; nrow < rows; nrow++){
			for(int ncol = 0; ncol < cols; ncol++){
				groundPolygons[im][nrow][ncol][0] = Point(cornersX.at<int>(nrow, ncol), cornersY.at<int>(nrow, ncol));
				groundPolygons[im][nrow][ncol][1] = Point(cornersX.at<int>(nrow, ncol+1), cornersY.at<int>(nrow, ncol+1));
				groundPolygons[im][nrow][ncol][2] = Point(cornersX.at<int>(nrow+1, ncol+1), cornersY.at<int>(nrow+1, ncol+1));
				groundPolygons[im][nrow][ncol][3] = Point(cornersX.at<int>(nrow+1, ncol), cornersY.at<int>(nrow+1, ncol));
			}
		}
	}
}

cv::Point3f Camera::computePointProjection(cv::Point2f imPoint, int cameraInd){
	Mat point(3, 1, CV_32FC1);
	point.at<float>(0) = imPoint.x * cameraZ * tan(angleX/2);
	point.at<float>(1) = imPoint.y * cameraZ * tan(angleY/2);
	point.at<float>(2) = -cameraZ;

	Mat rot(cameraOrig[cameraInd], Rect(Point(0, 0), Point(3, 3)));
	Mat trans(cameraOrig[cameraInd], Rect(Point(3, 0), Point(4, 3)));
	point = rot * point;
	Mat planeABC(groundPlane, Rect(Point(0, 0), Point(1, 3)));
	Mat planeD(groundPlane, Rect(Point(0, 3), Point(1, 4)));
	Mat a = (-planeABC.t() * trans - planeD) / (planeABC.t() * point);
	point = trans + point * a;
	Point3f ret(point.at<float>(0), point.at<float>(1), point.at<float>(2));
	return ret;
}

void Camera::learn(cv::Mat samples, int label){

}

cv::Mat Camera::selectPolygonPixels(std::vector<cv::Point2i> polygon, const cv::Mat& image){
	Mat mask(image.rows, image.cols, CV_8UC1, Scalar(0));
	int polyCnt[] = {polygon.size()};
	const Point2i* points[] = {polygon.data()};
	//Point2i array
	fillPoly(mask, points, polyCnt, 1, Scalar(1));
	int count = countNonZero(mask);
	Mat ret(count, 1, CV_8UC3);
	int idx = 0;
	for(int row = 0; row < image.rows; row++){
		for(int col = 0; col < image.cols; col++){
			if(mask.at<int>(row, col) != 0){
				ret.at<Vec3b>(idx) = image.at<Vec3b>(row, col);
			}
		}
	}
	return ret;
}

void Camera::learnFromDir(boost::filesystem::path dir){
	filesystem::directory_iterator endIt;
	for(filesystem::directory_iterator dirIt(dir); dirIt != endIt; dirIt++){
		if(dirIt->path().filename().string().find(".xml") != string::npos){
			TiXmlDocument data(dirIt->path().string());
			if(!data.LoadFile()){
				throw "Bad data file";
			}
			TiXmlElement* pAnnotation = data.FirstChildElement("annotation");
			if(!pAnnotation){
				throw "Bad data file - no annotation entry";
			}
			TiXmlElement* pFile = pAnnotation->FirstChildElement("filename");
			if(!pFile){
				throw "Bad data file - no filename entry";
			}
			Mat image = imread(dir.string() + pFile->GetText());
			if(image.data == NULL){
				throw "Bad image file";
			}
			TiXmlElement* pObject = pAnnotation->FirstChildElement("object");
			while(pObject){

				TiXmlElement* pPolygon = pObject->FirstChildElement("polygon");
				if(!pPolygon){
					throw "Bad data file - no polygon inside object";
				}
				vector<Point2i> poly;

				TiXmlElement* pPt = pPolygon->FirstChildElement("pt");
				while(pPt){
					int x = atoi(pPt->FirstChildElement("x")->GetText());
					int y = atoi(pPt->FirstChildElement("y")->GetText());
					poly.push_back(Point2i(x, y));
					pPt = pPt->NextSiblingElement("pt");
				}

				TiXmlElement* pAttributes = pObject->FirstChildElement("attributes");
				if(!pAttributes){
					throw "Bad data file - no object attributes";
				}
				string labelText = pAttributes->GetText();
				int label = 0;
				for(int i = 0; i < labels.size(); i++){
					if(labelText == labels[i]){
						label = i;
						break;
					}
				}

				Mat samples = selectPolygonPixels(poly, image);
				learn(samples, label);

				pObject = pObject->NextSiblingElement("object");
			}
		}
	}
}

cv::Mat Camera::classifySlidingWindow(cv::Mat image){

}

//Run as separate thread
void Camera::cameraThread(){

}

void Camera::readSettings(TiXmlElement* settings){
	if(settings->QueryIntAttribute("number", &numCameras) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of cameras";
	}
	if(settings->QueryIntAttribute("rows", &numRows) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of rows";
	}
	if(settings->QueryIntAttribute("cols", &numCols) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of cols";
	}
	if(settings->QueryIntAttribute("angle_x", &angleX) != TIXML_SUCCESS){
		throw "Bad settings file - wrong angle x";
	}
	if(settings->QueryIntAttribute("angle_y", &angleY) != TIXML_SUCCESS){
		throw "Bad settings file - wrong angle y";
	}
	if(settings->QueryIntAttribute("camera_z", &cameraZ) != TIXML_SUCCESS){
		throw "Bad settings file - wrong camera z";
	}

	cacheEnabled = true;
	TiXmlElement* pPtr = settings->FirstChildElement("cache");
	if(!pPtr){
		throw "Bad settings file - no cache setting";
	}
	pPtr->QueryBoolAttribute("enabled", &cacheEnabled);

	pPtr = settings->FirstChildElement("svm");
	if(!pPtr){
		throw "Bad settings file - no svm settings";
	}

	int svmType;
	string tmp;
	pPtr->QueryStringAttribute("type", &tmp);
	if(tmp == "C_SVC"){
		svmType = CvSVM::C_SVC;
	}
	else if(tmp == "NU_SVC"){
		svmType = CvSVM::NU_SVC;
	}
	else if(tmp == "ONE_CLASS"){
		svmType = CvSVM::ONE_CLASS;
	}
	else{
		throw "Bad settings file - wrong SVM type";
	}

	int kernelType;
	TiXmlElement* svmPtr = pPtr->FirstChildElement("kernelType");
	if(!svmPtr){
		throw "Bad settings file - no kernel type";
	}
	svmPtr->QueryStringAttribute("value", &tmp);
	if(tmp == "LINEAR"){
		kernelType = CvSVM::LINEAR;
	}
	else if(tmp == "POLY"){
		kernelType = CvSVM::POLY;
	}
	else if(tmp == "RBF"){
		kernelType = CvSVM::RBF;
	}
	else if(tmp == "SIGMOID"){
		kernelType = CvSVM::SIGMOID;
	}
	else{
		throw "Bad settings file - wrong kernel type";
	}

	bins = 256;
	svmPtr = pPtr->FirstChildElement("bins");
	if(!svmPtr){
		throw "Bad settings file - no bins number";
	}
	svmPtr->QueryIntAttribute("value", &bins);

	double gamma = 0.5;
	svmPtr = pPtr->FirstChildElement("gamma");
	if(!svmPtr){
		throw "Bad settings file - no gamma value";
	}
	svmPtr->QueryDoubleAttribute("value", &gamma);

	double degree = 2;
	svmPtr = pPtr->FirstChildElement("degree");
	if(!svmPtr){
		throw "Bad settings file - no degree value";
	}
	svmPtr->QueryDoubleAttribute("value", &degree);

	classifyGrid = 40;
	pPtr = settings->FirstChildElement("classification");
	if(!pPtr){
		throw "Bad settings file - no classification settings";
	}
	pPtr->QueryIntAttribute("grid", &classifyGrid);

	pPtr = settings->FirstChildElement("learning");
	if(!pPtr){
		throw "Bad settings file - no learning settings";
	}
	pPtr->QueryStringAttribute("dir", &tmp);
	learningDir = tmp;

	pPtr = settings->FirstChildElement("labels");
	TiXmlElement* pLabel = pPtr->FirstChildElement("label");
	while(pLabel){
		string text;
		int id;
		pLabel->QueryStringAttribute("text", &text);
		pLabel->QueryIntAttribute("id", &id);
		if(labels.size() <= id){
			labels.resize(id + 1);
		}
		labels[id] = text;
		pLabel = pLabel->NextSiblingElement("label");
	}

	pPtr = settings->FirstChildElement("sensor");
	for(int i = 0; i < numCameras; i++){
		if(!pPtr){
			throw "Bad settings file - no sensor settings";
		}
		pPtr->QueryStringAttribute("id", &tmp);
		int idx = 0;
		if(tmp == "left"){
			idx = LEFT_CAMERA;
		}
		else if(tmp == "right"){
			idx = RIGHT_CAMERA;
		}
		else{
			throw "Bad settings file - wrong camera id";
		}
		TiXmlElement* posPtr = pPtr->FirstChildElement("position");
		if(!posPtr){
			throw "Bad settings file - no position of sensor";
		}
		stringstream tmpStr(posPtr->Value());
		cameraOrig[idx] = Mat(4, 4, CV_32FC1);
		for(int row = 0; row < 4; row++){
			for(int col = 0; col < 4; col++){
				float tmpVal;
				tmpStr >> tmpVal;
				cameraOrig[idx].at<float>(row, col) = tmpVal;
			}
		}
		pPtr = pPtr->NextSiblingElement("sensor");
	}

	groundPlane = Mat(4, 1, CV_32FC1);
	pPtr = settings->FirstChildElement("ground_plane");
	if(!pPtr){
		throw "Bad settings file - no ground plane equation";
	}
	stringstream tmpStr(pPtr->Value());
	for(int i = 0; i < 4; i++){
		float tmpVal;
		tmpStr >> tmpVal;
		groundPlane.at<float>(i) = tmpVal;
	}

	svmParams = CvSVMParams();	//default values
	svmParams.kernel_type = kernelType;
	svmParams.svm_type = svmType;
	svmParams.degree = degree;
	svmParams.gamma = gamma;
}

void Camera::readCache(boost::filesystem::path cacheFile){
	entries.clear();
	TiXmlDocument doc(cacheFile.c_str());
	if(!doc.LoadFile()){
		throw "Could not load cache file";
	}
	TiXmlElement* pDatabase = doc.FirstChildElement("database");
	if(!pDatabase){
		throw "Bad cache file - database not found";
	}
	TiXmlElement* pEntry = pDatabase->FirstChildElement("entry");
	while(pEntry){
		Entry entry;
		entry.descriptor = Mat(bins, 1, CV_32FC1);
		pEntry->QueryIntAttribute("label", &entry.label);
		stringstream tmpStr(pEntry->GetText());
		for(int i = 0; i < bins; i++){
			float tmp;
			tmpStr >> tmp;
			entry.descriptor.at<float>(i) = tmp;
		}
		entries.push_back(entry);
		pEntry = pEntry->NextSiblingElement("entry");
	}
}

void Camera::saveCache(boost::filesystem::path cacheFile){
	TiXmlDocument doc;
	TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "UTF-8", "");
	doc.LinkEndChild(decl);
	TiXmlElement* pDatabase = new TiXmlElement("database");
	doc.LinkEndChild(pDatabase);
	for(int entr = 0; entr < entries.size(); entr++){
		TiXmlElement* pEntry = new TiXmlElement("entry");
		pDatabase->LinkEndChild(pEntry);
		pEntry->SetAttribute("label", entries[entr].label);
		stringstream tmpStr;
		for(int i = 0; i < bins; i++){
			tmpStr << entries[entr].descriptor.at<float>(i) << " ";
		}
		pEntry->SetValue(tmpStr.str());
	}
	doc.SaveFile(cacheFile.c_str());
}

//Returns constraints map and inserts time of data from cameras fetch
const cv::Mat Camera::getConstraints(int* timestamp){

}

//CV_8UC3 2x640x480: left, right image
const std::vector<cv::Mat> Camera::getData(){
	//empty matrix
	vector<Mat> ret;
	ret.push_back(Mat(numRows, numCols, CV_8UC3));
	ret.push_back(Mat(numRows, numCols, CV_8UC3));
	return ret;
}

void Camera::open(){

}

void Camera::close(){

}

bool Camera::isOpen(){
	return true;
}

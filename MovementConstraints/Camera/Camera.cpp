/*
 * Camera.cpp
 *
 *  Created on: 08-07-2013
 *      Author: jachu
 */

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
//STL
#include <cmath>
#include <sstream>
#include <algorithm>
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
#define X_STEP 100
#define Y_STEP 100
#define X_RES 50
#define Y_RES 50
#define PLANE_Z -100
#define DRIVABLE_LABEL 3
#define LEFT_CAMERA 0
#define RIGHT_CAMERA 1

#define CHANNELS_USED 2
#define SAMPLE_PACK 1500

#define NO_CUDA

using namespace cv;
using namespace gpu;
using namespace std;


Camera::Camera(MovementConstraints* imovementConstraints, TiXmlElement* settings) :
		movementConstraints(imovementConstraints)
{
	if(!settings){
		throw "Bad settings file - entry Camera not found";
	}
	readSettings(settings);

	computeImagePolygons();

#ifndef NO_CUDA
	cout << "Available CUDA devices: " << getCudaEnabledDeviceCount() << endl;
	setDevice(0);
	DeviceInfo gpuInfo;
	cout << "Version: " << gpuInfo.majorVersion() << "." << gpuInfo.minorVersion() << endl;
	cout << "Number of processors: " << gpuInfo.multiProcessorCount() << endl;
#endif //NO_CUDA
}

Camera::~Camera(){
	for(int i = 0; i < hierClassifiers.size(); i++){
		delete hierClassifiers[i];
	}
}

void Camera::computeConstraints(std::vector<cv::Mat> image){
	cout << "Computing constraints" << endl;
	constraints = Mat(X_RES, Y_RES, CV_32FC1, Scalar(0));
	Mat constrNorm(X_RES, Y_RES, CV_32FC1, Scalar(0));
	namedWindow("original");
	namedWindow("prob asphalt");
	namedWindow("constraints");
	for(int c = 0; c < image.size(); c++){
		vector<Mat> classRes = hierClassifiers[c]->classify(image[c]);
		Mat regions(image[c].rows, image[c].cols, CV_32SC1, Scalar(-1));
		for(int xInd = 0; xInd < X_RES; xInd++){
			for(int yInd = 0; yInd < Y_RES; yInd++){
				selectPolygonPixels(imagePolygons[c][xInd][yInd],
									xInd*X_RES + yInd,
									regions);
				/*float probDriv = 0;
				for(int l = 0; l < classRes.size(); l++){
					if(l == DRIVABLE_LABEL){
						Mat tmp(classRes[l].rows, classRes[l].rows, CV_32FC1, Scalar(0));
						classRes[l].copyTo(tmp, mask);
						probDriv += sum(tmp)[0];
					}
				}
				constraints.at<float>(xInd, yInd) = probDriv;*/
			}
		}
		//cout << "adding probabilities" << endl;
		for(int row = 0; row < image[c].rows; row++){
			for(int col = 0; col < image[c].cols; col++){
				if(regions.at<int>(row, col) != -1){
					int xInd = regions.at<int>(row, col) / X_RES;
					int yInd = regions.at<int>(row, col) % X_RES;
					//cout << "adding " << classRes[DRIVABLE_LABEL].at<float>(row, col) << endl;
					constraints.at<float>(yInd, xInd) += classRes[DRIVABLE_LABEL].at<float>(row, col);
					constrNorm.at<float>(yInd, xInd) += 1;
				}
			}
		}
		cout << "normalizing constraints" << endl;
		for(int xInd = 0; xInd < X_RES; xInd++){
			for(int yInd = 0; yInd < Y_RES; yInd++){
				if(constrNorm.at<float>(yInd, xInd) != 0){
					constraints.at<float>(yInd, xInd) /= constrNorm.at<float>(yInd, xInd);
				}
			}
		}

		cout << "building test image" << endl;
		Mat test(image[c].rows, image[c].cols, CV_32FC1);
		for(int xInd = 0; xInd < X_RES; xInd++){
			for(int yInd = 0; yInd < Y_RES; yInd++){
				selectPolygonPixels(imagePolygons[c][xInd][yInd],
									constraints.at<float>(yInd, xInd),
									test);
			}
		}
		imshow("original", image[c]);
		imshow("prob asphalt", classRes[DRIVABLE_LABEL]);
		imshow("constraints", test);
		waitKey();
	}
}

void Camera::computeImagePolygons(){
	cout << "Computing image polygons" << endl;
	//namedWindow("test");
	imagePolygons.clear();
	imagePolygons.assign(numCameras,
						vector<vector<vector<Point2f> > >(X_RES,
							vector<vector<Point2f> >(Y_RES,
									vector<Point2f>())));
	for(int c = 0; c < numCameras; c++){
		Mat rotVec;
		Rodrigues(Mat(cameraOrigGlobal[c].inv(DECOMP_SVD), Rect(0, 0, 3, 3)), rotVec);
		Mat transVec(cameraOrigGlobal[c].inv(DECOMP_SVD), Rect(3, 0, 1, 3));

		//cout << "cameraOrigGlobal[c].inv(DECOMP_SVD) = " << cameraOrigGlobal[c].inv(DECOMP_SVD) << endl;
		//cout << "rotVec = " << rotVec << endl;
		//cout << "transVec = " << transVec << endl;

		//from front left corner
		float xStart = X_RES*X_STEP;
		float xStep = -X_STEP;
		float yStart = Y_RES*Y_STEP/2;
		float yStep = -Y_STEP;
		for(int xInd = 0; xInd < X_RES; xInd++){
			for(int yInd = 0; yInd < Y_RES; yInd++){
				vector<Point3f> points;
				int ind[][2] = {{0, 0},
								{1, 0},
								{1, 1},
								{0, 1}};
				for(int i = 0; i < 4; i++){
					points.push_back(Point3f(	xStart + (xInd + ind[i][0])*xStep,
												yStart + (yInd + ind[i][1])*yStep,
												PLANE_Z));
				}
				//cout << "Polygon glob: " << endl;
				//for(int v = 0; v < points.size(); v++){
				//	cout << points[v] << endl;
				//}
				vector<Point2f> ret;
				projectPoints(	points,
								rotVec,
								transVec,
								cameraMatrix[c],
								distCoeffs[c],
								ret);
				//Mat test(480, 640, CV_8UC1, Scalar(0));
				//cout << "Polygon image: " << endl;
				//for(int v = 0; v < ret.size(); v++){
				//	cout << ret[v] << endl;
				//}
				//selectPolygonPixels(ret, 1, test);
				//imshow("test", hierClassifiers.front()->colorSegments(test));
				//waitKey();
				imagePolygons[c][xInd][yInd]= ret;
			}
		}
	}
	cout << "End computing image polygons" << endl;
}

std::vector<cv::Point2f> Camera::computePointProjection(const std::vector<cv::Point3f>& spPoints,
														int cameraInd)
{
	vector<Point2f> ret;
	projectPoints(	spPoints,
					Matx<float, 3, 1>(0, 0, 0),
					Matx<float, 3, 1>(0, 0, 0),
					cameraMatrix[cameraInd],
					distCoeffs[cameraInd],
					ret);
	return ret;
}

std::vector<cv::Point3f> Camera::computePointReprojection(	const std::vector<cv::Point2f>& imPoints,
															int cameraInd)
{

}

/*void Camera::addToLearnDatabase(cv::Mat samples, int label){
	Mat data[] = {samples};
	int channels[] = {0};
	int histSize[] = {bins};
	float range[] = {0, 256};
	const float* ranges[] = {range};
	Mat hist(1, bins, CV_32FC1);
	Entry newEntry;
	newEntry.label = label;
	newEntry.descriptor = Mat(1, CHANNELS_USED*bins, CV_32FC1);

	for(int i = 0; i < CHANNELS_USED; i++){
		channels[0] = i;
		calcHist(data, 1, channels, Mat(), hist, 1, histSize, ranges);
		hist.copyTo(newEntry.descriptor.colRange(bins*i, bins*(i + 1) - 1));
	}
	normalize(hist, hist, 1, 0, NORM_L1, -1);
	entries.push_back(newEntry);
}

void Camera::clearLearnDatabase(){
	entries.clear();
}

void Camera::learn(){
	if(entries.size() > 0){
		Mat allHist(entries.size(), entries[0].descriptor.cols, CV_32FC1);
		Mat allLabels(entries.size(), 1, CV_8UC1);
		for(int i = 0; i < entries.size(); i++){
			entries[i].descriptor.copyTo(allHist.rowRange(i, i));
			allLabels.at<unsigned char>(i) = entries[i].label;
		}
		svm.train(allHist, allLabels, Mat(), Mat(), svmParams);
	}
}*/

int Camera::selectPolygonPixels(std::vector<cv::Point2i> polygon, float regionId, cv::Mat& regionsOnImage){
	int polyCnt[] = {polygon.size()};
	const Point2i* points[] = {polygon.data()};
	//Point2i array
	fillPoly(regionsOnImage, points, polyCnt, 1, Scalar(regionId));
	int count = regionsOnImage.rows * regionsOnImage.cols - countNonZero(regionsOnImage - Scalar(regionId));

	return count;
}

int Camera::selectPolygonPixels(std::vector<cv::Point2f> polygon, float regionId, cv::Mat& regionsOnImage){
	vector<Point2i> tmp;
	for(int v = 0; v < polygon.size(); v++){
		tmp.push_back(Point2i(cvRound(polygon[v].x), cvRound(polygon[v].y)));
	}
	return selectPolygonPixels(tmp, regionId, regionsOnImage);
}


void Camera::learnFromDir(boost::filesystem::path dir){
	//namedWindow("segments");
	//namedWindow("original");
	vector<Entry> dataset;
	vector<double> weights;
	filesystem::directory_iterator endIt;
	for(filesystem::directory_iterator dirIt(dir); dirIt != endIt; dirIt++){
		if(dirIt->path().filename().string().find(".xml") != string::npos){
			cout << "Reading XML file: " << dirIt->path().string() << endl;
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

			Mat image = imread(dir.string() + string("/") + pFile->GetText());
			if(image.data == NULL){
				throw "Bad image file";
			}

			//loading map
			int imageNum;
			sscanf(pFile->GetText(), "camera%d.jpg", &imageNum);
			Mat terrain;
			char terrainFilename[200];
			sprintf(terrainFilename, "%smap%03d.log", (dir.string() + string("/")).c_str(), imageNum);
			ifstream terrainFile(terrainFilename);
			if(terrainFile.is_open() == false){
				throw "No map file";
			}
			double tmp;
			while(!terrainFile.eof()){
				Mat terrainPoint(1, 5, CV_32FC1);	//x, y, z, distance, intensity
				for(int i = 0; i < 5; i++){
					terrainFile >> tmp;
					terrainPoint.at<float>(0, i) = tmp;
				}
				terrain.push_back(terrainPoint);
			}
			terrain = terrain.t();

			Mat manualRegionsOnImage(image.rows, image.cols, CV_32SC1, Scalar(0));
			int manualRegionsCount = 0;

			Mat autoRegionsOnImage = hierClassifiers.front()->segmentImage(image);
			map<int, int> mapRegionIdToLabel;

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

				mapRegionIdToLabel[++manualRegionsCount] = label;
				//cout << "Selecting polygon pixels for label " << labels[label] <<  endl;
				selectPolygonPixels(poly, manualRegionsCount, manualRegionsOnImage);
				//cout << "End selecting" << endl;

				pObject = pObject->NextSiblingElement("object");
			}

			map<int, int> assignedManualId = hierClassifiers.front()->assignManualId(autoRegionsOnImage, manualRegionsOnImage);
			//imshow("original", image);
			//imshow("segments", hierClassifiers.front()->colorSegments(autoRegionsOnImage));
			//waitKey();

			vector<Entry> newData = hierClassifiers.front()->extractEntries(image, terrain, autoRegionsOnImage);
			for(int e = 0; e < newData.size(); e++){
				if(mapRegionIdToLabel.count(assignedManualId[newData[e].imageId]) > 0){
					newData[e].label = mapRegionIdToLabel[assignedManualId[newData[e].imageId]];
					dataset.push_back(newData[e]);
				}
			}
		}
	}

	map<int, double> sizeOfLabels;
	for(int e = 0; e < dataset.size(); e++){
		sizeOfLabels[dataset[e].label] += dataset[e].weight;
	}
	for(int e = 0; e < dataset.size(); e++){
		dataset[e].weight /= sizeOfLabels[dataset[e].label]*sizeOfLabels.size();
		//dataset[e].weight = (dataset[e].label == 1 ? 1 : 100);
	}

	sizeOfLabels.clear();
	for(int e = 0; e < dataset.size(); e++){
		sizeOfLabels[dataset[e].label] += dataset[e].weight;
	}
	for(map<int, double>::iterator it = sizeOfLabels.begin(); it != sizeOfLabels.end(); ++it){
		cout << "label " << it->first << ", weight = " << it->second << endl;
	}

	if(crossValidate){
		hierClassifiers.front()->crossValidateSVMs(dataset);
	}
	hierClassifiers.front()->train(dataset, labels.size());

	if(cacheEnabled){
		saveCache("cameraCache");
	}
}

void Camera::classifyFromDir(boost::filesystem::path dir){
	cout << "Classifying" << endl;
	for(int l = 0; l < labels.size(); l++){
		namedWindow(labels[l]);
	}
	namedWindow("segments");
	namedWindow("original");
	namedWindow("colored");

	vector<vector<int> > classResultsPix(labels.size(), vector<int>(labels.size(), 0));
	vector<vector<int> > classResultsSeg(labels.size(), vector<int>(labels.size(), 0));

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
			Mat image = imread(dir.string() + string("/") + pFile->GetText());
			if(image.data == NULL){
				throw "Bad image file";
			}

			//loading map
			int imageNum;
			sscanf(pFile->GetText(), "camera%d.jpg", &imageNum);
			Mat terrain;
			char terrainFilename[200];
			sprintf(terrainFilename, "%smap%03d.log", (dir.string() + string("/")).c_str(), imageNum);
			ifstream terrainFile(terrainFilename);
			if(terrainFile.is_open() == false){
				throw "No map file";
			}
			double tmp;
			while(!terrainFile.eof()){
				Mat terrainPoint(1, 5, CV_32FC1);	//x, y, z, distance, intensity
				for(int i = 0; i < 5; i++){
					terrainFile >> tmp;
					terrainPoint.at<float>(0, i) = tmp;
				}
				terrain.push_back(terrainPoint);
			}
			terrain = terrain.t();

			Mat manualRegionsOnImage(image.rows, image.cols, CV_32SC1, Scalar(0));
			int manualRegionsCount = 0;
			map<int, int> mapRegionIdToLabel;

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

				mapRegionIdToLabel[++manualRegionsCount] = label;
				selectPolygonPixels(poly, manualRegionsCount, manualRegionsOnImage);

				pObject = pObject->NextSiblingElement("object");
			}
			Mat autoSegmented = hierClassifiers.front()->segmentImage(image, 200);
			map<int, int> assignedManualId = hierClassifiers.front()->assignManualId(autoSegmented, manualRegionsOnImage);
			imshow("original", image);
			imshow("segments", hierClassifiers.front()->colorSegments(autoSegmented));

			vector<vector<int> > curClassResultsPix(labels.size(), vector<int>(labels.size(), 0));
			vector<vector<int> > curClassResultsSeg(labels.size(), vector<int>(labels.size(), 0));

			vector<Mat> classificationResult = hierClassifiers.front()->classify(image, terrain, autoSegmented);
			for(int l = 0; l < labels.size(); l++){
				double minVal, maxVal;
				minMaxIdx(classificationResult[l], &minVal, &maxVal);
				cout << labels[l] << ", min = " << minVal << ", max = " << maxVal << endl;
				imshow(labels[l], classificationResult[l]);
			}
			vector<Scalar> colors;
			colors.push_back(Scalar(0, 255, 0));	//grass - green
			colors.push_back(Scalar(0, 0, 255));	//wood - red
			colors.push_back(Scalar(0, 255, 255));	//yellow - ceramic
			colors.push_back(Scalar(255, 0, 0));	//blue - asphalt
			Mat coloredOriginal(image.rows, image.cols, CV_8UC3);
			Mat bestLabels(image.rows, image.cols, CV_32SC1, Scalar(0));
			Mat bestScore(image.rows, image.cols, CV_32FC1, Scalar(-1));
			for(int l = 0; l < labels.size(); l++){
				Mat cmp;
				compare(bestScore, classificationResult[l], cmp, CMP_LE);
				bestLabels.setTo(l, cmp);
				bestScore = max(bestScore, classificationResult[l]);
			}
			for(int l = 0; l < labels.size(); l++){
				coloredOriginal.setTo(colors[l], bestLabels == l);
			}
			imshow("colored", coloredOriginal * 0.25 + image * 0.75);

			set<int> counted;
			for(int r = 0; r < image.rows; r++){
				for(int c = 0; c < image.cols; c++){
					int pred = bestLabels.at<int>(r, c);
					if(mapRegionIdToLabel.count(assignedManualId[autoSegmented.at<int>(r, c)]) > 0){
						if(counted.count(autoSegmented.at<int>(r, c)) == 0){
							int groundTrue = mapRegionIdToLabel[assignedManualId[autoSegmented.at<int>(r, c)]];
							curClassResultsSeg[groundTrue][pred]++;
							counted.insert(autoSegmented.at<int>(r, c));
						}
					}
					if(mapRegionIdToLabel.count(manualRegionsOnImage.at<int>(r, c)) > 0){
						int groundTrue = mapRegionIdToLabel[manualRegionsOnImage.at<int>(r, c)];
						curClassResultsPix[groundTrue][pred]++;
					}
				}
			}

			for(int t = 0; t < labels.size(); t++){
				for(int p = 0; p < labels.size(); p++){
					classResultsPix[t][p] += curClassResultsPix[t][p];
					classResultsSeg[t][p] += curClassResultsSeg[t][p];
				}
			}

			cout << "Current frame pixel results: " << endl;
			for(int t = 0; t < labels.size(); t++){
				cout << "true = " << t << ": ";
				for(int p = 0; p < labels.size(); p++){
					cout << curClassResultsPix[t][p] << ", ";
				}
				cout << endl;
			}

			cout << "Current frame segment results: " << endl;
			for(int t = 0; t < labels.size(); t++){
				cout << "true = " << t << ": ";
				for(int p = 0; p < labels.size(); p++){
					cout << curClassResultsSeg[t][p] << ", ";
				}
				cout << endl;
			}

			waitKey();
		}
	}

	cout << "General pixel results: " << endl;
	for(int t = 0; t < labels.size(); t++){
		cout << "true = " << t << ": ";
		for(int p = 0; p < labels.size(); p++){
			cout << classResultsPix[t][p] << ", ";
		}
		cout << endl;
	}

	cout << "General segment results: " << endl;
	for(int t = 0; t < labels.size(); t++){
		cout << "true = " << t << ": ";
		for(int p = 0; p < labels.size(); p++){
			cout << classResultsSeg[t][p] << ", ";
		}
		cout << endl;
	}

	cout << "End classifying" << endl;
}

/*cv::Mat Camera::classifySlidingWindow(cv::Mat image){
	//wxDateTime StartTime = wxDateTime::UNow();

	const int rows = image.rows;
	const int cols = image.cols;
	const int step = classifyGrid;

	GpuMat imageHSV(rows, cols, CV_8UC3);
	GpuMat imageH(rows, cols, CV_8UC1);
	GpuMat imageS(rows, cols, CV_8UC1);
	GpuMat imageV(rows, cols, CV_8UC1);
	GpuMat out[] = {imageH, imageS, imageV};

	imageHSV.upload(image);
	cvtColor(imageHSV, imageHSV, CV_BGR2HSV);
	split(imageHSV, out);

	//wxDateTime UploadTime = wxDateTime::UNow();

	vector<Mat> votes(labels.size());

	for (int i = 0; i < (int)votes.size(); i++)
	{
		// Sepatate Mat for each entry
		votes[i] = Mat(image.rows, image.cols, CV_8U, Scalar(0));
	}

	GpuMat*** entries = new GpuMat**[rows/step];
	for(int row = 0; row < rows/step; row++){
		entries[row] = new GpuMat*[cols/step];
	}
	for(int row = 0; row < rows/step; row++){
		for(int col = 0; col < cols/step; col++){
			entries[row][col] = new GpuMat(1, 2*bins, CV_32SC1);
		}
	}

	cout << "Calculating histograms" << endl;
	GpuMat buf(1, bins, CV_32SC1);
	for (int row = step; row <= rows; row += step)
	{
		for (int col = step; col <= cols; col += step)
		{
			const int MinC = col - step;
		    const int MaxC = col;
		    const int MinR = row - step;
		    const int MaxR = row;

		    //cout << "MinX: " << MinX << " MinY: " << MinY << " MaxX " << MaxX << " MaxY " << MaxY << "\n";
		    GpuMat RoiH = GpuMat(imageH, Rect(Point(MinC, MinR), Point(MaxC, MaxR)));
		    GpuMat RoiS = GpuMat(imageS, Rect(Point(MinC, MinR), Point(MaxC, MaxR)));

		    //cout << "Calculating hist for row = " << row << ", col = " << col << endl;
			GenerateColorHistHSVGpu(RoiH, RoiS, *entries[(row - 1)/step][(col - 1)/step], buf);

		}
	}

	//wxDateTime HistTime = wxDateTime::UNow();

	cout << "Classifing" << endl;

    Mat histSum(1, 2*bins, CV_32FC1);
    GpuMat histSumGpu(1, 2*bins, CV_32FC1);
    buf = GpuMat(1, 2*bins, CV_32FC1);
	for (int row = classifyGrid; row <= rows; row += step)
	{
		for (int col = classifyGrid; col <= cols; col += step)
		{
			const int MinC = col - classifyGrid;
		    const int MaxC = col;
		    const int MinR = row - classifyGrid;
		    const int MaxR = row;

		    int idxR = (row - 1)/step;
		    int idxC = (col - 1)/step;
		    int subGrids = classifyGrid/step;
		    histSumGpu = Scalar(0);
		    for(int subRow = idxR - subGrids + 1; subRow <= idxR; subRow++){
			    for(int subCol = idxC - subGrids + 1; subCol <= idxC; subCol++){
			    	add(histSumGpu, *entries[subRow][subCol], histSumGpu);
			    }
		    }
		    normalize(histSum, histSum, 1, 0, NORM_L1, -1, buf);
		    histSumGpu.download(histSum);
		    unsigned int predictedLabel = 0;
		    predictedLabel = svm.predict(histSum);
		    //cout << WordPredictedLabel << endl;
		    //EndTimeClass = wxDateTime::UNow();

		    Mat Mask = Mat(rows, cols, CV_8U, Scalar(0));
		    rectangle(Mask, Point(MinC, MinR), Point(MaxC, MaxR), Scalar(0x1), CV_FILLED);
		    votes[predictedLabel] +=  Mask;
		}
	}

	for(int row = 0; row < rows/step; row++){
		for(int col = 0; col < cols/step; col++){
			delete entries[row][col];
		}
	}
	for(int row = 0; row < rows/step; row++){
		delete[] entries[row];
	}
	delete[] entries;

	//wxDateTime ClassTime = wxDateTime::UNow();

	//cout << "Uploading and converting time: " << (UploadTime - StartTime).Format(wxString::FromAscii("%M:%S:%l")).ToAscii().data() << "\n";
	//cout << "Calculating histograms time: " << (HistTime - UploadTime).Format(wxString::FromAscii("%M:%S:%l")).ToAscii().data() << "\n";
	//cout << "Classifing time: " << (ClassTime - HistTime).Format(wxString::FromAscii("%M:%S:%l")).ToAscii().data() << "\n";

}

void Camera::GenerateColorHistHSVGpu(
		const cv::gpu::GpuMat& ImageH,
		const cv::gpu::GpuMat& ImageS,
		cv::gpu::GpuMat& result,
		cv::gpu::GpuMat& buf)
{
	GpuMat HistH(1, bins, CV_32SC1);
	GpuMat HistS(1, bins, CV_32SC1);

	if(bins != 256){
		throw "Number of bins must be equal to 256";
	}

	calcHist(ImageH, HistH, buf);
	calcHist(ImageS, HistS, buf);

	GpuMat partH = result.colRange(0, bins);
	GpuMat partS = result.colRange(bins, 2*bins);

	HistH.copyTo(partH);
	HistS.copyTo(partS);

	result.convertTo(result, CV_32F);
}*/

//Run as separate thread
void Camera::cameraThread(){

}



void Camera::readSettings(TiXmlElement* settings){
	string tmp;

	if(settings->QueryIntAttribute("number", &numCameras) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of cameras";
	}
	if(settings->QueryIntAttribute("rows", &numRows) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of rows";
	}
	if(settings->QueryIntAttribute("cols", &numCols) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of cols";
	}

	cacheEnabled = true;
	TiXmlElement* pPtr = settings->FirstChildElement("cache");
	if(!pPtr){
		throw "Bad settings file - no cache setting for Camera";
	}
	pPtr->QueryBoolAttribute("enabled", &cacheEnabled);

	crossValidate = false;

	pPtr = settings->FirstChildElement("learning");
	if(!pPtr){
		throw "Bad settings file - no learning settings";
	}
	pPtr->QueryStringAttribute("dir", &tmp);
	learningDir = tmp;

	pPtr = settings->FirstChildElement("labels");
	if(!pPtr){
		throw "Bad settings file - no labels settings";
	}
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
	cameraOrigGlobal.resize(numCameras);
	cameraOrigLaser.resize(numCameras);
	cameraMatrix.resize(numCameras);
	distCoeffs.resize(numCameras);
	hierClassifiers.resize(numCameras);
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

		cameraOrigGlobal[idx] = readMatrixSettings(pPtr, "position_global", 4, 4);
		cameraOrigLaser[idx] = readMatrixSettings(pPtr, "position_laser", 4, 4);
		cameraMatrix[idx] = readMatrixSettings(pPtr, "camera_matrix", 3, 3);
		distCoeffs[idx] = readMatrixSettings(pPtr, "dist_coeffs", 1, 5);


		hierClassifiers[idx] = new HierClassifier(cameraMatrix[idx]);

		pPtr = pPtr->NextSiblingElement("sensor");
	}

	pPtr = settings->FirstChildElement("HierClassifier");
	if(!pPtr){
		throw "Bad settings file - no HierClassifier settings";
	}
	for(int i = 0; i < hierClassifiers.size(); i++){
		hierClassifiers[i]->loadSettings(pPtr);
	}

	groundPlane = readMatrixSettings(settings, "ground_plane_global", 4, 1);

	/*svmParams = CvSVMParams();	//default values
	svmParams.kernel_type = kernelType;
	svmParams.svm_type = svmType;
	svmParams.degree = degree;
	svmParams.gamma = gamma;*/
}

cv::Mat Camera::readMatrixSettings(TiXmlElement* parent, const char* node, int rows, int cols){
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

void Camera::readCache(boost::filesystem::path cacheFile){
	/*entries.clear();
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
		int descLength;
		pEntry->QueryIntAttribute("desc_length", &descLength);
		entry.descriptor = Mat(descLength, 1, CV_32FC1);
		pEntry->QueryIntAttribute("label", &entry.label);
		stringstream tmpStr(pEntry->GetText());
		for(int i = 0; i < descLength; i++){
			float tmp;
			tmpStr >> tmp;
			entry.descriptor.at<float>(i) = tmp;
		}
		entries.push_back(entry);
		pEntry = pEntry->NextSiblingElement("entry");
	}*/
}

void Camera::saveCache(boost::filesystem::path cacheFile){
	/*TiXmlDocument doc;
	TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "UTF-8", "");
	doc.LinkEndChild(decl);
	TiXmlElement* pDatabase = new TiXmlElement("database");
	doc.LinkEndChild(pDatabase);
	for(int entr = 0; entr < entries.size(); entr++){
		TiXmlElement* pEntry = new TiXmlElement("entry");
		pDatabase->LinkEndChild(pEntry);
		pEntry->SetAttribute("label", entries[entr].label);
		stringstream tmpStr;
		for(int i = 0; i < entries[entr].descriptor.cols; i++){
			tmpStr << entries[entr].descriptor.at<float>(i) << " ";
		}
		//TODO correct using TiXmlText
		pEntry->SetValue(tmpStr.str());
	}
	doc.SaveFile(cacheFile.c_str());*/
	for(int c = 0; c < numCameras; c++){
		char buffer[10];
		sprintf(buffer, "%02d.xml", c);
		hierClassifiers[c]->saveCache(cacheFile.string() + buffer);
	}
}

//Returns constraints map and inserts time of data from cameras fetch
const cv::Mat Camera::getConstraints(int* timestamp){

}

//CV_8UC3 2x640x480: left, right image
const std::vector<cv::Mat> Camera::getData(){
	//empty matrix
	vector<Mat> ret;
	for(int i = 0; i < cameras.size(); i++){
		cameras[i].grab();
	}
	for(int i = 0; i < cameras.size(); i++){
		Mat tmp;
		cameras[i].retrieve(tmp);
		ret.push_back(tmp);
	}
	return ret;
}

void Camera::open(std::vector<std::string> device){
	cameras.resize(device.size());
	for(int i = 0; i < device.size(); i++){
		cout << "Opening device: " << device[i] << endl;
		cameras[i].open(0);
		if(!cameras[i].isOpened()){
			throw "Cannot open camera device";
		}
	}
}

void Camera::close(){
	cout << "Closing cameras" << endl;
	for(int i = 0; i < cameras.size(); i++){
		cameras[i].release();
	}
	cameras.clear();
}

bool Camera::isOpen(){
	return true;
}

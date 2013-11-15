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
#include "UnionFind.h"

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

#define CHANNELS_USED 2
#define SAMPLE_PACK 1500

#define NO_CUDA

using namespace cv;
using namespace gpu;
using namespace std;

Scalar colors[] = {
		Scalar(0xFF, 0x00, 0x00), //Red
		Scalar(0xFF, 0xFF, 0xFF),	//White
		Scalar(0x00, 0xFF, 0xFF), //Cyan
		Scalar(0xC0, 0xC0, 0xC0), //Silver
		Scalar(0x00, 0x00, 0xFF), //Blue
		Scalar(0x80, 0x80, 0x80), //Gray
		Scalar(0x00, 0x00, 0xA0), //DarkBlue
		Scalar(0x00, 0x00, 0x00), //Black
		Scalar(0xAD, 0xD8, 0xE6), //LightBlue
		Scalar(0xFF, 0xA5, 0x00), //Orange
		Scalar(0x80, 0x00, 0x80), //Purple
		Scalar(0xA5, 0x2A, 0x2A), //Brown
		Scalar(0xFF, 0xFF, 0x00), //Yellow
		Scalar(0x80, 0x00, 0x00), //Maroon
		Scalar(0x00, 0xFF, 0x00), //Lime
		Scalar(0x00, 0x80, 0x00), //Green
		Scalar(0xFF, 0x00, 0xFF), //Magenta
		Scalar(0x80, 0x80, 0x00) //Olive
};


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

#ifndef NO_CUDA
	cout << "Available CUDA devices: " << getCudaEnabledDeviceCount() << endl;
	setDevice(0);
	DeviceInfo gpuInfo;
	cout << "Version: " << gpuInfo.majorVersion() << "." << gpuInfo.minorVersion() << endl;
	cout << "Number of processors: " << gpuInfo.multiProcessorCount() << endl;
#endif //NO_CUDA
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

	Mat rot(cameraOrigGlobal[cameraInd], Rect(Point(0, 0), Point(3, 3)));
	Mat trans(cameraOrigGlobal[cameraInd], Rect(Point(3, 0), Point(4, 3)));
	point = rot * point;
	Mat planeABC(groundPlane, Rect(Point(0, 0), Point(1, 3)));
	Mat planeD(groundPlane, Rect(Point(0, 3), Point(1, 4)));
	Mat a = (-planeABC.t() * trans - planeD) / (planeABC.t() * point);
	point = trans + point * a;
	Point3f ret(point.at<float>(0), point.at<float>(1), point.at<float>(2));
	return ret;
}

void Camera::addToLearnDatabase(cv::Mat samples, int label){
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

				//divide samples into smaller packs
				Mat samples = selectPolygonPixels(poly, image);
				for(int i = 0; i < samples.cols/SAMPLE_PACK; i++){
					addToLearnDatabase(samples.colRange(i*SAMPLE_PACK, (i + 1)*SAMPLE_PACK - 1), label);
				}
				//taking care of remainder samples - if too small, reject
				int rem = samples.cols % SAMPLE_PACK;
				if(rem >= SAMPLE_PACK/4){
					addToLearnDatabase(samples.colRange(samples.cols - rem, samples.cols - 1), label);
				}

				pObject = pObject->NextSiblingElement("object");
			}
		}
	}
	learn();
}

cv::Mat Camera::classifySlidingWindow(cv::Mat image){
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
}

//Run as separate thread
void Camera::cameraThread(){

}

struct Edge{
	int i, j;
	float weight;
	Edge() {}
	Edge(int ii, int ij, float iweight) : i(ii), j(ij), weight(iweight) {}
};

bool operator<(const Edge& left, const Edge& right){
	return left.weight < right.weight;
}


cv::Mat Camera::segment(cv::Mat image){
	Mat imageR(image.rows, image.cols, CV_32FC1);
	Mat imageG(image.rows, image.cols, CV_32FC1);
	Mat imageB(image.rows, image.cols, CV_32FC1);
	Mat imageChannels[] = {imageR, imageG, imageB};
	Mat imageFloat(image.rows, image.cols, CV_32FC3);
	float k = 200;
	int minSize = 20;
	int nchannels = 3;
	int nrows = image.rows;
	int ncols = image.cols;
	int nhood[][2] = {{-1, 1},
					{1, 0},
					{1, 1},
					{0, 1}};
	/*int nhood[][2] = {{1, 0},
					{0, 1}};*/
	//cout << "Size of nhood " << sizeof(nhood)/sizeof(nhood[0]) << endl;

	image.convertTo(imageFloat, CV_32F);
	GaussianBlur(imageFloat, imageFloat, Size(7, 7), 0.8);
	split(imageFloat, imageChannels);

	imshow("original", imageFloat/255);

	vector<Mat> segments;

	{
		//cout << "Channel " << ch << endl;
		segments.push_back(Mat(nrows, ncols, CV_32SC1));
		vector<Edge> edges;
		for(int r = 0; r < nrows; r++){
			for(int c = 0; c < ncols; c++){
				for(int nh = 0; nh < sizeof(nhood)/sizeof(nhood[0]); nh++){
					if((r + nhood[nh][0] < nrows) && (r + nhood[nh][0] >= 0) &&
							(c + nhood[nh][1] < ncols) && (c + nhood[nh][1] >= 0))
					{
						float diffAll = 0;
						for(int ch = 0; ch < nchannels; ch++){
							float diff = abs(imageChannels[ch].at<float>(r, c) - imageChannels[ch].at<float>(r + nhood[nh][0], c + nhood[nh][1]));
							diffAll += diff*diff;
						}
						diffAll = sqrt(diffAll);
						edges.push_back(Edge(c + ncols*r, c + nhood[nh][1] + ncols*(r + nhood[nh][0]), diffAll));
						//if(edges.back().i == 567768 || edges.back().j == 567768){
						//	cout << "diff = abs(" << (int)imageChannels[ch].at<unsigned char>(r, c) << " - " << (int)imageChannels[ch].at<unsigned char>(r + nhood[nh][0], c + nhood[nh][1]) << ") = " << diff << endl;
						//}
					}
				}
			}
		}
		stable_sort(edges.begin(), edges.end()); //possible improvement by bin sorting
		cout << "Largest differece = " << edges[edges.size() - 1].weight <<
				", between (" << edges[edges.size() - 1].i << ", " << edges[edges.size() - 1].j <<
				")" << endl;
		UnionFind sets(nrows * ncols);
		vector<float> intDiff;
		intDiff.assign(nrows * ncols, 0);
		for(vector<Edge>::iterator it = edges.begin(); it != edges.end(); it++){
			int iRoot = sets.findSet(it->i);
			int jRoot = sets.findSet(it->j);
			//cout << "i = " << it->i << ", j = " << it->j << ", weight = " << it->weight << endl;
			if(iRoot != jRoot){
				//cout << "intDiff[iRoot] + (float)k/sizes[iRoot] = " << intDiff[iRoot] << " + " << (float)k/sizes[iRoot] << " = " << intDiff[iRoot] + (float)k/sizes[iRoot] << endl;
				//cout << "intDiff[jRoot] + (float)k/sizes[jRoot] = " << intDiff[jRoot] << " + " << (float)k/sizes[jRoot] << " = " << intDiff[jRoot] + (float)k/sizes[jRoot] << endl;
				if(min(intDiff[iRoot] + (float)k/sets.size(iRoot), intDiff[jRoot] + (float)k/sets.size(jRoot))
						>=
						it->weight)
				{
					//cout << "union " << min(intDiff[iRoot] + (float)k/sizes[iRoot], intDiff[jRoot] + (float)k/sizes[jRoot]) << " >= " << it->weight << endl;
					int newRoot = sets.unionSets(iRoot, jRoot);
					intDiff[newRoot] = it->weight;
				}
			}
		}
		for(vector<Edge>::iterator it = edges.begin(); it != edges.end(); it++){
			int iRoot = sets.findSet(it->i);
			int jRoot = sets.findSet(it->j);
			if((iRoot != jRoot) && ((sets.size(iRoot) < minSize) || (sets.size(jRoot) < minSize))){
				sets.unionSets(iRoot, jRoot);
			}
		}
		set<int> numElements;
		for(int r = 0; r < nrows; r++){
			for(int c = 0; c < ncols; c++){
				segments.back().at<int>(r, c) = sets.findSet(c + ncols*r);
				numElements.insert(sets.findSet(c + ncols*r));
			}
		}
		cout << "number of elements = " << numElements.size() << endl;
	}

	Mat finSegments(nrows, ncols, CV_32SC1);
	UnionFind sets(nrows * ncols);
	vector<Edge> edges;
	for(int r = 0; r < nrows; r++){
		for(int c = 0; c < ncols; c++){
			for(int nh = 0; nh < sizeof(nhood)/sizeof(nhood[0]); nh++){
				if((r + nhood[nh][0] < nrows) && (r + nhood[nh][0] >= 0) &&
						(c + nhood[nh][1] < ncols) && (c + nhood[nh][1] >= 0))
				{
					edges.push_back(Edge(c + ncols*r, c + nhood[nh][1] + ncols*(r + nhood[nh][0]), 0));
				}
			}
		}
	}
	for(vector<Edge>::iterator it = edges.begin(); it != edges.end(); it++){
		bool areOneSegment = true;
		for(int ch = 0; ch < segments.size(); ch++){
			if(segments[ch].at<int>(it->i / ncols, it->i % ncols) != segments[ch].at<int>(it->j / ncols, it->j % ncols)){
				areOneSegment = false;
				break;
			}
		}
		if(areOneSegment){
			sets.unionSets(it->i, it->j);
		}
	}
	for(int r = 0; r < nrows; r++){
		for(int c = 0; c < ncols; c++){
			finSegments.at<int>(r, c) = sets.findSet(c + ncols*r);
		}
	}

	map<int, int> colorMap;
	int ind = 0;
	for(int r = 0; r < nrows; r++){
		for(int c = 0; c < ncols; c++){
			finSegments.at<int>(r, c) = sets.findSet(c + ncols*r);
			if(colorMap.count(finSegments.at<int>(r, c)) == 0){
				colorMap.insert(pair<int, int>(finSegments.at<int>(r, c), (ind++) % (sizeof(colors)/sizeof(colors[0]))));
			}
		}
	}
	cout << "Found " << colorMap.size() << " segments" << endl;
	Mat segImage(nrows, ncols, CV_8UC3);
	for(map<int, int>::iterator it = colorMap.begin(); it != colorMap.end(); it++){
		Mat mask = (finSegments == it->first);

		segImage.setTo(colors[it->second], mask);
	}
	imshow("segmented", segImage);

	return finSegments;
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

		pPtr = pPtr->NextSiblingElement("sensor");
	}

	groundPlane = readMatrixSettings(settings, "ground_plane_global", 4, 1);

	svmParams = CvSVMParams();	//default values
	svmParams.kernel_type = kernelType;
	svmParams.svm_type = svmType;
	svmParams.degree = degree;
	svmParams.gamma = gamma;
}

cv::Mat Camera::readMatrixSettings(TiXmlElement* parent, const char* node, int rows, int cols){
	TiXmlElement* ptr = parent->FirstChildElement(node);
	if(!ptr){
		throw (string("Bad settings file - no ") + string(node)).c_str();
	}
	stringstream tmpStr(ptr->Value());
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

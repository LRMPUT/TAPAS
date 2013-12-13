/*
 * HierClassifier.cpp
 *
 *  Created on: 22-10-2013
 *      Author: jachu
 */

//OpenCV
#include <opencv2/opencv.hpp>
//STL
#include <cmath>
#include <algorithm>
//RobotsIntellect
#include "HierClassifier.h"
#include "UnionFind.h"
#include "ClassifierSVM.h"

#define HIST_SIZE_H 4
#define HIST_SIZE_S 4
#define HIST_SIZE_V 16
#define COVAR_SIZE 9
#define MEAN_SIZE 3

using namespace std;
using namespace cv;
using namespace boost;

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

cv::Mat HierClassifier::projectPointsTo3D(cv::Mat disparity){
	Mat ret;
	Mat Q = Mat::eye(4, 4, CV_32FC1);
	cameraMatrix.copyTo(Mat(Q, Rect(0, 0, 3, 3)));
	reprojectImageTo3D(disparity, ret, Q);
	return ret;
}

cv::Mat HierClassifier::projectPointsTo2D(cv::Mat _3dImage){
	Mat ret;
	projectPoints(_3dImage, ret, (0,0,0), (0,0,0), cameraMatrix, Mat());
	return ret;
}

//---------------MISCELLANEOUS----------------

HierClassifier::HierClassifier(cv::Mat icameraMatrix) :
	cacheEnabled(false)
{

}

/** \brief Loads settings from XML structure.

*/
HierClassifier::HierClassifier(cv::Mat icameraMatrix, TiXmlElement* settings) :
	cacheEnabled(false)
{
	loadSettings(settings);
}

HierClassifier::~HierClassifier(){
	for(int c = 0; c < weakClassifiersSet.size(); c++){
		delete weakClassifiersSet[c];
	}
	for(int c = 0; c < classifiers.size(); c++){
		delete classifiers[c];
	}
}

/** \brief Loads settings from XML structure.

*/
void HierClassifier::loadSettings(TiXmlElement* settings){
	TiXmlElement* pPtr = settings->FirstChildElement("cache");
	if(!pPtr){
		throw "Bad settings file - no cache setting for HierClassifier";
	}
	pPtr->QueryBoolAttribute("enabled", &cacheEnabled);

	pPtr = settings->FirstChildElement("segmentation");
	if(!pPtr){
		throw "Bad settings file - no segmentation setting for HierClassifier";
	}
	pPtr->QueryFloatAttribute("k", &kSegment);
	pPtr->QueryIntAttribute("min_size", &minSizeSegment);

	numWeakClassifiers = 3;
	int begCol = 0;
	weakClassInfo.push_back(WeakClassifierInfo(begCol, begCol + HIST_SIZE_H*HIST_SIZE_S));	//histogram HS
	begCol += HIST_SIZE_H*HIST_SIZE_S;
	weakClassInfo.push_back(WeakClassifierInfo(begCol, begCol + HIST_SIZE_V));	//histogram V
	begCol += HIST_SIZE_V;
	weakClassInfo.push_back(WeakClassifierInfo(begCol, begCol + COVAR_SIZE + MEAN_SIZE));	//statistics HSV
	//weakClassInfo.push_back(WeakClassifierInfo());	//statistics height
	//weakClassInfo.push_back(WeakClassifierInfo());	//statistics intensity
	//weakClassInfo.push_back(WeakClassifierInfo());	//geometric properties

	numIterations = 10;

	weakClassifiersSet.resize(numWeakClassifiers);
	for(int c = 0; c < numWeakClassifiers; c++){
		weakClassifiersSet[c] = new ClassifierSVM();
	}

	pPtr = settings->FirstChildElement("ClassifierSVM");
	if(!pPtr){
		throw "Bad settings file - no ClassifierSVM setting";
	}
	for(int i = 0; i < weakClassifiersSet.size(); i++){
		if(weakClassifiersSet[i]->type() == Classifier::SVM){
			weakClassifiersSet[i]->loadSettings(pPtr);
		}
	}
}

void HierClassifier::saveCache(boost::filesystem::path file){

}

void HierClassifier::loadCache(boost::filesystem::path file){

}

//---------------COMPUTING----------------

/** Trains classifier. Outline of algorithm:
 * 		1. Initialize dataWeights = 1/data.size()
 * 		2. for t = 0 : numIterations
 * 		3.		trains classifiers
 * 		4.		choose best classsifier
 * 		5.		compute weight for that classifier
 * 		6.		update dataWeights
 */
void HierClassifier::train(const std::vector<Entry>& data, int inumLabels){
	cout << "HierClassifier::train, numIterations = " << numIterations << endl;

	numLabels = inumLabels;
	//initializing weights
	vector<double> dataWeights;
	dataWeights.assign(data.size(), (double)1/data.size());

	//constructing dataset for each classifier without copying data
	vector<vector<Entry> > dataClassifiers;
	dataClassifiers.resize(numWeakClassifiers);
	for(int c = 0; c < numWeakClassifiers; c++){
		dataClassifiers[c].resize(data.size());
		for(int e = 0; e < data.size(); e++){
			Entry tmp;
			tmp.descriptor = data[e].descriptor.colRange(
													weakClassInfo[c].descBeg,
													weakClassInfo[c].descEnd);
			tmp.label = data[e].label;
			dataClassifiers[c][e] = tmp;
		}
	}

	//main loop
	for(int t = 0; t < numIterations; t++){
		cout << "Iteration " << t << endl;
		int maxIndex = 0;
		double maxVal = 0;
		for(int c = 0; c < numWeakClassifiers; c++){
			cout << "Training classifier " << c << endl;
			//training
			weakClassifiersSet[c]->train(dataClassifiers[c], dataWeights);

			//evaluating
			double score = 0;
			for(int e = 0; e < dataClassifiers[c].size(); e++){
				cout << "Entry " << e << ", weight " << dataWeights[e] << ": ";
				Mat probEst = weakClassifiersSet[c]->classify(dataClassifiers[c][e].descriptor);
				probEst *= 2;
				probEst -= 1;
				for(int l = 0; l < numLabels; l++){
					int ind = (l == dataClassifiers[c][e].label ? 1 : -1);
					cout << ind << " (" << probEst.at<float>(l) << "), ";
					score += dataWeights[e]*probEst.at<float>(l)*ind/numLabels;
				}
				cout << endl;
			}
			if(score > maxVal){
				maxVal = score;
				maxIndex = c;
			}
		}

		//computing weight for best classifer
		//TODO compute accurate value of alpha
		double alpha = 0.5*log((1 + maxVal)/(1 - maxVal));
		weights.push_back(alpha);

		cout << "Adding classifier " << maxIndex << ", alpha = " << alpha << endl;
		//adding classifier
		classifiers.push_back(weakClassifiersSet[maxIndex]->copy());

		cout << "recomputing dataWeights" << endl;
		//recomputing dataWeights
		double sum = 0;
		for(int e = 0; e < dataClassifiers[maxIndex].size(); e++){
			Mat probEst = classifiers.back()->classify(dataClassifiers[maxIndex][e].descriptor);
			probEst *= 2;
			probEst -= 1;
			double score = 0;
			for(int l = 0; l < numLabels; l++){
				int ind = (l == dataClassifiers[maxIndex][e].label ? 1 : -1);
				score += probEst.at<float>(l)*ind;
			}
			dataWeights[e] *= exp(-alpha*score);
			sum += dataWeights[e];
		}
		for(int e = 0; e < dataWeights.size(); e++){
			dataWeights[e] /= sum;
		}
		//waitKey();
	}
}

/**	\brief
	@return Matrix of probabilites of belonging to certain class.
*/
std::vector<cv::Mat> HierClassifier::classify(cv::Mat image,
							  	  	  	  	  cv::Mat terrain)
{
	Mat regionsOnImage = segmentImage(image);
	vector<Entry> entries = extractEntries(image, terrain, regionsOnImage);
	map<int, int> imageIdToEntry;
	for(int e = 0; e < entries.size(); e++){
		imageIdToEntry[entries[e].imageId] = e;
	}
	Mat result(entries.size(), numLabels, CV_32FC1);
	for(int c = 0; c < classifiers.size(); c++){
		for(int e = 0; e < entries.size(); e++){
			result.row(e) = result.row(e) + weights[c]*classifiers[c]->classify(entries[e].descriptor);
		}
	}
	vector<Mat> ret;
	ret.resize(numLabels);
	for(int l = 0; l < numLabels; l++){
		ret[l] = Mat(image.rows, image.cols, CV_32FC1);
	}
	for(int l = 0; l < numLabels; l++){
		for(int r = 0; r < image.rows; r++){
			for(int c = 0; c < image.cols; c++){
				ret[l].at<float>(r, c) = result.at<float>(imageIdToEntry[regionsOnImage.at<int>(r, c)], l);
			}
		}
	}
	return ret;
}

struct Pixel{
	int r, c;
	int imageId;
	Pixel(){}
	Pixel(int ir, int ic, int iimageId) :
		r(ir), c(ic), imageId(iimageId)
	{}
};

bool operator<(const Pixel& left, const Pixel& right){
	return (left.imageId < right.imageId);
}

/** \brief Extracts entries from labeled data.

*/
std::vector<Entry> HierClassifier::extractEntries(	cv::Mat imageBGR,
													cv::Mat terrain,
													cv::Mat regionsOnImage)
{
	Mat imageHSV;
	cvtColor(imageBGR, imageHSV, CV_BGR2HSV);
	vector<Pixel> pixels;
	pixels.resize(imageHSV.rows * imageHSV.cols);
	for(int r = 0; r < imageHSV.rows; r++){
		for(int c = 0; c < imageHSV.cols; c++){
			pixels[r * imageHSV.cols + c] = Pixel(r, c, regionsOnImage.at<int>(r, c));
		}
	}
	sort(pixels.begin(), pixels.end());
	vector<Entry> ret;
	//vector<Mat> values;
	//vector<Mat> histogramsHS;
	//vector<Mat> histogramsV;
	//vector<Mat> statisticsHSV;
	int end = 0;
	while(end < pixels.size()){
		Mat values, histogramHS, histogramV, statisticsHSV;
		int beg = end;
		while(pixels[beg].imageId == pixels[end].imageId){
			end++;
		}
		//cout << "segment id = " << pixels[beg].imageId << ", beg = " << beg << ", end = " << end << endl;
		values = Mat(1, end - beg, CV_8UC3);
		for(int p = beg; p < end; p++){
			values.at<Vec3b>(p - beg) = imageHSV.at<Vec3b>(pixels[p].r, pixels[p].c);
		}
		int channelsHS[] = {0, 1};
		float rangeH[] = {0, 256};
		float rangeS[] = {0, 360};
		const float* rangesHS[] = {rangeH, rangeS};
		int sizeHS[] = {HIST_SIZE_H, HIST_SIZE_S};
		int channelsV[] = {2};
		float rangeV[] = {0, 256};
		const float* rangesV[] = {rangeV};
		int sizeV[] = {HIST_SIZE_V};
		calcHist(&values, 1, channelsHS, Mat(), histogramHS, 2, sizeHS, rangesHS);
		calcHist(&values, 1, channelsV, Mat(), histogramV, 1, sizeV, rangesV);
		histogramHS = histogramHS.reshape(0, 1);
		histogramV = histogramV.reshape(0, 1);
		normalize(histogramHS, histogramHS);
		normalize(histogramV, histogramV);
		//cout << "V size = " << histogramV.size() << ", HS size = " << histogramHS.size() << endl;


		values = values.reshape(1, 3);
		//cout << "values size = " << values.size() << endl;
		Mat covarHSV;
		Mat meanHSV;
		calcCovarMatrix(values,
						covarHSV,
						meanHSV,
						CV_COVAR_NORMAL | CV_COVAR_SCALE | CV_COVAR_COLS);
		//cout << "Calculated covar matrix" << endl;
		covarHSV = covarHSV.reshape(0, 1);
		meanHSV = meanHSV.reshape(0, 1);
		normalize(covarHSV, covarHSV);
		normalize(meanHSV, meanHSV);

		//cout << "Entry " << ret.size() << endl;
		//cout << "histHS = " << histogramHS << endl;
		//cout << "histV = " << histogramV << endl;
		//cout << "covarHSV = " << covarHSV << endl;
		//cout << "meanHSV = " << meanHSV << endl;
		Entry tmp;
		tmp.imageId = pixels[beg].imageId;
		tmp.descriptor = Mat(1, HIST_SIZE_H*HIST_SIZE_S + HIST_SIZE_V + COVAR_SIZE + MEAN_SIZE, CV_32FC1);
		int begCol = 0;
		histogramHS.copyTo(tmp.descriptor.colRange(begCol, begCol + HIST_SIZE_H*HIST_SIZE_S));
		begCol += HIST_SIZE_H*HIST_SIZE_S;
		histogramV.copyTo(tmp.descriptor.colRange(begCol, begCol + HIST_SIZE_V));
		begCol += HIST_SIZE_V;
		covarHSV.copyTo(tmp.descriptor.colRange(begCol, begCol + COVAR_SIZE));
		begCol += COVAR_SIZE;
		meanHSV.copyTo(tmp.descriptor.colRange(begCol, begCol + MEAN_SIZE));
		ret.push_back(tmp);
	}
	return ret;
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


cv::Mat HierClassifier::segmentImage(cv::Mat image){
	Mat imageR(image.rows, image.cols, CV_32FC1);
	Mat imageG(image.rows, image.cols, CV_32FC1);
	Mat imageB(image.rows, image.cols, CV_32FC1);
	Mat imageChannels[] = {imageR, imageG, imageB};
	Mat imageFloat(image.rows, image.cols, CV_32FC3);
	int nchannels = 3;
	int nhood[][2] = {{-1, 1},
					{1, 0},
					{1, 1},
					{0, 1}};
	/*int nhood[][2] = {{1, 0},
					{0, 1}};*/
	//cout << "Size of nhood " << sizeof(nhood)/sizeof(nhood[0]) << endl;
	//cout << "rows: " << image.rows << ", cols: " << image.cols << endl;

	image.convertTo(imageFloat, CV_32F);
	//resize(imageFloat, imageFloat, Size(320, 240));
	GaussianBlur(imageFloat, imageFloat, Size(7, 7), 0.8);
	split(imageFloat, imageChannels);

	int nrows = imageFloat.rows;
	int ncols = imageFloat.cols;

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
		sort(edges.begin(), edges.end()); //possible improvement by bin sorting
		//cout << "Largest differece = " << edges[edges.size() - 1].weight <<
		//		", between (" << edges[edges.size() - 1].i << ", " << edges[edges.size() - 1].j <<
		//		")" << endl;
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
				if(min(intDiff[iRoot] + (float)kSegment/sets.size(iRoot), intDiff[jRoot] + (float)kSegment/sets.size(jRoot))
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
			if((iRoot != jRoot) && ((sets.size(iRoot) < minSizeSegment) || (sets.size(jRoot) < minSizeSegment))){
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
		//cout << "number of elements = " << numElements.size() << endl;
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

	return finSegments;
}

Mat HierClassifier::colorSegments(const Mat segments){
	int nrows = segments.rows;
	int ncols = segments.cols;
	Mat ret(nrows, ncols, CV_8UC3);

	map<int, int> colorMap;
	int ind = 0;
	for(int r = 0; r < nrows; r++){
		for(int c = 0; c < ncols; c++){
			if(colorMap.count(segments.at<int>(r, c)) == 0){
				colorMap.insert(pair<int, int>(segments.at<int>(r, c), (ind++) % (sizeof(colors)/sizeof(colors[0]))));
			}
		}
	}

	for(map<int, int>::iterator it = colorMap.begin(); it != colorMap.end(); it++){
		Mat mask = (segments == it->first);

		ret.setTo(colors[it->second], mask);
	}

	return ret;
}


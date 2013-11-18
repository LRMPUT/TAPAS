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

using namespace std;
using namespace cv;

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

}

cv::Mat HierClassifier::projectPointsTo2D(cv::Mat _3dImage){

}

//---------------MISCELLANEOUS----------------

HierClassifier::HierClassifier(cv::Mat icameraMatrix){

}

/** \brief Loads settings from XML structure.

*/
HierClassifier::HierClassifier(cv::Mat icameraMatrix, TiXmlElement* settings){

}

HierClassifier::~HierClassifier(){

}

/** \brief Loads settings from XML structure.

*/
void HierClassifier::loadSettings(TiXmlElement* settings){

}

void HierClassifier::saveCache(boost::filesystem::path file){

}

void HierClassifier::loadCache(boost::filesystem::path file){

}

//---------------COMPUTING----------------

void HierClassifier::train(std::vector<Entry> data){

}

/**	\brief
	@return Matrix of probabilites of belonging to certain class.
*/
std::vector<cv::Mat> HierClassifier::classify(cv::Mat image,
							  	  	  	  	  cv::Mat terrain)
{

}

/** \brief Extracts entities from labeled data.

*/
std::vector<Entry> HierClassifier::extractEntities(	cv::Mat image,
													cv::Mat terrain,
													cv::Mat regionsOnImage)
{

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


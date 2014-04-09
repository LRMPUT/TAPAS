/*
 * Crf.h
 *
 *  Created on: 20-03-2014
 *      Author: jachu
 */

#ifndef CRF_H_
#define CRF_H_

#include <vector>
#include <opencv2/opencv.hpp>

struct Edge{
	int i, j;
	double theta;
};

struct Graph{
	std::vector<Edge> edges;
	cv::Mat thetasNode;
};

class Crf{

public:
	Crf();
	cv::Mat optimize(cv::Mat prob);
};


#endif /* CRF_H_ */

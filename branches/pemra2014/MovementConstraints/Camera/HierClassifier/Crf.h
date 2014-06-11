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


class Crf{

public:
	struct Edge{
		int i, j;
		Edge() {}
		Edge(int ii, int ij) : i(ii), j(ij) {}
	};

	struct Graph{
		std::vector<double> thetaE;
		std::vector<std::vector<Edge> > edgesList;

		std::vector<double> thetaN;
		//std::vector<int> nodes;
	};


private:

	std::vector<int> labelsList;

	std::vector<double> conditionalDist(int node,
										const Graph& graph,
										const std::vector<int>& labels,
										const std::vector<std::vector<double> >& probObs,
										const std::vector<cv::Mat>& features);
	double functionEdge(const Edge& edge,
						const Graph& graph,
						const std::vector<int>& labels,
						const std::vector<std::vector<double> >& probObs,
						const std::vector<cv::Mat>& features);
	double functionNode(int node,
						const Graph& graph,
						const std::vector<int>& labels,
						const std::vector<std::vector<double> >& probObs,
						const std::vector<cv::Mat>& features);
public:
	Crf(const std::vector<int> ilabelsList);
	std::vector<double> optimize(	const Graph& graph,
									std::vector<double> probObs);
	std::vector<std::vector<double> > gibbsSampler(const Graph& graph,
												const std::vector<int>& initLab,
												const std::vector<std::vector<double> >& probObs,
												const std::vector<cv::Mat>& features);
};


#endif /* CRF_H_ */

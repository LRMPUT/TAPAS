/*
 * Crf.cpp
 *
 *  Created on: 20-03-2014
 *      Author: jachu
 */

#ifndef CRF_H
#define CRF_H

#include <chrono>
#include <random>

#include "Crf.h"

using namespace std;

std::vector<std::vector<double> > Crf::gibbsSampler(const Graph& graph,
												const std::vector<int>& initLab,
												const std::vector<std::vector<double> >& probObs,
												const std::vector<cv::Mat>& features)
{
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	default_random_engine generator(seed);
	uniform_real_distribution<double> uniRealDist(0.0, 1.0);
	//uniform_int_distribution<int> uniIntDist(0, labelsList.size() - 1);

	vector<vector<double> > ret(labelsList.size(), vector<double>(graph.edgesList.size(), 0));
	vector<int> curLab(initLab);
	static const int burnIn = 500;
	static const int numIter = 2000;
	for(int i = 0; i < numIter; i++){
		cout << "Iteration " << i << endl;
		for(int n = 0; n < graph.edgesList.size(); n++){
			vector<double> condDist = conditionalDist(n,
														graph,
														curLab,
														probObs,
														features);
			/*for(int l = 0; l < labelsList.size(); l++){
				if(abs(condDist[l] - probObs[l][n]) > 0.001){
					cout << "condDist[" << l << "] = " << condDist[l] << ", probObs[" << l << "][" << n << "] = " << probObs[l][n] << endl;
				}
			}*/
			int newLab = 0;
			double acceptRand = uniRealDist(generator);
			while((acceptRand > condDist[newLab]) && (newLab < condDist.size() - 1)){
				acceptRand -= condDist[newLab];
				newLab++;
			}
			curLab[n] = newLab;
			if(i >= burnIn){
				ret[newLab][n] += 1;
			}
		}
	}
	for(int n = 0; n < graph.edgesList.size(); n++){
		for(int l = 0; l < labelsList.size(); l++){
			ret[l][n] /= numIter - burnIn;
		}
	}
	return ret;
}

std::vector<double> Crf::conditionalDist(int node,
										const Graph& graph,
										const std::vector<int>& labels,
										const std::vector<std::vector<double> >& probObs,
										const std::vector<cv::Mat>& features)
{
	double normConst = 0;
	vector<double> ret(labelsList.size(), 0);
	vector<int> locLabels(labels);
	for(int l = 0; l < labelsList.size(); l++){
		double prob = 0;
		locLabels[node] = labelsList[l];
		prob += functionNode(node,
							graph,
							locLabels,
							probObs,
							features);
		for(int e = 0; e < graph.edgesList[node].size(); e++){
			prob += functionEdge(graph.edgesList[node][e],
								graph,
								locLabels,
								probObs,
								features);
		}

		prob = exp(prob);
		ret[l] = prob;
		normConst += prob;
	}

	for(int l = 0; l < labelsList.size(); l++){
		ret[l] /= normConst;
	}
	return ret;
}

double Crf::functionEdge(const Edge& edge,
					const Graph& graph,
					const std::vector<int>& labels,
					const std::vector<std::vector<double> >& probObs,
					const std::vector<cv::Mat>& features)
{
	double ret = 0;
	int labI = labels[edge.i];
	int labJ = labels[edge.j];
	vector<double> funG(features.front().cols, 0);

	//function
	static const double beta = 0.01;
	if(labI != labJ){
		for(int f = 0; f < funG.size(); f++){
			double diffF = (features[edge.i].at<float>(f) - features[edge.j].at<float>(f));
			funG[f] = exp(-beta*diffF*diffF);
		}
	}
	//end function

	for(int i = 0; i < funG.size(); i++){
		ret += funG[i]*graph.thetaE[i];
	}
	return ret;
}

double Crf::functionNode(int node,
					const Graph& graph,
					const std::vector<int>& labels,
					const std::vector<std::vector<double> >& probObs,
					const std::vector<cv::Mat>& features)
{
	double ret = 0;
	vector<double> funF;

	int curLabel = labels[node];
	int curLabelInd = -1;
	for(int l = 0; l < labelsList.size(); l++){
		if(curLabel == labelsList[l]){
			curLabelInd = l;
			break;
		}
	}

	//function
	funF.push_back(log(probObs[curLabelInd][node]));
	//end function

	for(int i = 0; i < funF.size(); i++){
		ret += funF[i]*graph.thetaN[i];
	}
	return ret;
}

Crf::Crf(const std::vector<int> ilabelsList) :
	labelsList(ilabelsList)
{

}

std::vector<double> Crf::optimize(	const Graph& graph,
								std::vector<double> probObs)
{

}

#endif

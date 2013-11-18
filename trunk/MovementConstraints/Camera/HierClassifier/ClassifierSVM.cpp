/*
 * ClassifierSVM.cpp
 *
 *  Created on: 22-10-2013
 *      Author: jachu
 */

#include "Classifier.h"
#include "ClassifierSVM.h"

using namespace cv;
using namespace std;

void ClassifierSVM::startup(){
	svmMainParams.svm_type = C_SVC;
	svmMainParams.cache_size = 32;
	svmMainParams.eps = 0.001;
	svmMainParams.probability = 1;
}

ClassifierSVM::ClassifierSVM(){
	startup();
}

/** \brief Loads settings from XML structure.

*/
ClassifierSVM::ClassifierSVM(TiXmlElement* settings){
	startup();
	loadSettings(settings);
}

ClassifierSVM::~ClassifierSVM(){

}

/** \brief Loads settings from XML structure.

*/
void ClassifierSVM::loadSettings(TiXmlElement* settings){

}

void ClassifierSVM::saveCache(boost::filesystem::path file){

}

void ClassifierSVM::loadCache(boost::filesystem::path file){

}

//---------------COMPUTING----------------
void ClassifierSVM::train(std::vector<Entry> entries){
	int descLen = entries.front().descriptor.cols;
	int numLabels = 0;
	labData = new svm_node*[entries.size()];
	for(int e = 0; e < entries.size(); e++){
		labData[e] = new svm_node[descLen + 1];
		for(int i = 0; i < descLen; i++){
			svm_node tmp;
			tmp.index = i;
			tmp.value = entries[e].descriptor.at<float>(i);
			labData[e][i] = tmp;
		}
		svm_node tmp;
		tmp.index = -1;
		labData[e][descLen] = tmp;
		numLabels = max(numLabels, entries[e].label);
	}
	numLabels++;

	dataLabels.resize(numLabels);
	numEntriesLabeled.assign(numLabels, 0);
	for(int l = 0; l < dataLabels.size(); l++){
		dataLabels[l] = new double[entries.size()];
		for(int e = 0; e < entries.size(); e++){
			dataLabels[l][e] = (entries[e].label == l ? 1 : -1);
		}
	}
	for(int e = 0; e < entries.size(); e++){
		numEntriesLabeled[entries[e].label]++;
	}

	svmMainParams.nr_weight = numLabels;
	svmsParams.resize(numLabels);
	svms.resize(numLabels);
	svmsProblems.resize(numLabels);
	for(int l = 0; l < numLabels; l++){
		svmsParams[l] = svmMainParams;
		svmsParams[l].weight_label = new int[2];
		svmsParams[l].weight = new double[2];
		svmsParams[l].weight_label[0] = 1;
		svmsParams[l].weight_label[1] = -1;
		svmsParams[l].weight[0] = (double)(entries.size() - numEntriesLabeled[l])/entries.size();
		svmsParams[l].weight[1] = (double)numEntriesLabeled[l]/entries.size();
		svmsProblems[l].l = numLabels;
		svmsProblems[l].y = dataLabels[l];
		svmsProblems[l].x = labData;
		if(svm_check_parameter(&svmsProblems[l], &svmsParams[l]) != NULL){
			throw "Bad svm params";
		}
		svms[l] = svm_train(&svmsProblems[l], &svmsParams[l]);
	}
}

cv::Mat ClassifierSVM::classify(cv::Mat features, cv::Mat prevOutput){
	Mat ret(1, svms.size(), CV_32FC1);
	double prob_est[2];
	svm_node* data = new svm_node[features.cols + 1];
	for(int i = 0; i < features.cols; i++){
		svm_node tmp;
		tmp.index = i;
		tmp.value = features.at<float>(i);
		data[i] = tmp;
	}
	svm_node tmp;
	tmp.index = -1;
	data[features.cols] = tmp;

	for(int l = 0; l < svms.size(); l++){
		svm_predict_probability(svms[l], data, prob_est);
		ret.at<float>(l) = prob_est[0];
	}

	delete[] data;
	return ret;
}

/*
 * ClassifierSVM.cpp
 *
 *  Created on: 22-10-2013
 *      Author: jachu
 */

//Boost
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include "Classifier.h"
#include "ClassifierSVM.h"


using namespace cv;
using namespace std;
using namespace boost;

void ClassifierSVM::startup(){
	numEntries = 0;

	svmMainParams.svm_type = C_SVC;
	svmMainParams.cache_size = 32;
	svmMainParams.eps = 0.001;
	svmMainParams.probability = 1;
}

void ClassifierSVM::clearData(){
	for(int i = 0; i < numEntries; i++){
		delete[] labData[i];
	}
	delete[] labData;

	for(int i = 0; i < dataLabels.size(); i++){
		delete[] dataLabels[i];
	}
	for(int i = 0; i < svmsParams.size(); i++){
		delete svmsParams[i].weight_label;
		delete svmsParams[i].weight;
	}
	for(int i = 0; i < svms.size(); i++){
		svm_free_and_destroy_model(&svms[i]);
	}
	dataLabels.clear();
	svmsParams.clear();
	svms.clear();
	svmsProblems.clear();
	numEntriesLabeled.clear();
	startup();
}

ClassifierSVM::ClassifierSVM() :
	Classifier(Classifier::SVM)
{
	startup();
}

/** \brief Loads settings from XML structure.

*/
ClassifierSVM::ClassifierSVM(TiXmlElement* settings) :
	Classifier(Classifier::SVM, settings)
{
	startup();
	loadSettings(settings);
}

ClassifierSVM::~ClassifierSVM(){
	clearData();
}

/** \brief Loads settings from XML structure.

*/
void ClassifierSVM::loadSettings(TiXmlElement* settings){
	string tmp;

	cacheEnabled = true;
	TiXmlElement* pPtr = settings->FirstChildElement("cache");
	if(!pPtr){
		throw "Bad settings file - no cache setting for ClassifierSVM";
	}
	pPtr->QueryBoolAttribute("enabled", &cacheEnabled);

	pPtr = settings->FirstChildElement("svm");
	if(!pPtr){
		throw "Bad settings file - no svm settings";
	}

	int svmType;
	pPtr->QueryStringAttribute("type", &tmp);
	if(tmp == "C_SVC"){
		svmType = C_SVC;
	}
	else if(tmp == "NU_SVC"){
		svmType = NU_SVC;
	}
	else if(tmp == "ONE_CLASS"){
		svmType = ONE_CLASS;
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
		kernelType = LINEAR;
	}
	else if(tmp == "POLY"){
		kernelType = POLY;
	}
	else if(tmp == "RBF"){
		kernelType = RBF;
	}
	else if(tmp == "SIGMOID"){
		kernelType = SIGMOID;
	}
	else{
		throw "Bad settings file - wrong kernel type";
	}

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

	svmMainParams.svm_type = svmType;
	svmMainParams.kernel_type = kernelType;
	svmMainParams.gamma = gamma;
	svmMainParams.degree = degree;
}

void ClassifierSVM::saveCache(boost::filesystem::path filePref){
	for(int i = 0; i < svms.size(); i++){
		svm_save_model((filePref.string() + (boost::format("%03d") % i).str()).c_str(), svms[i]);
	}
}

void ClassifierSVM::loadCache(boost::filesystem::path filePref){
	filesystem::directory_iterator endIt;
	for(filesystem::directory_iterator dirIt(filePref); dirIt != endIt; dirIt++){
		if(string(dirIt->path().filename().c_str()).find(filePref.filename().string()) != string::npos){
			svm_model* tmp = svm_load_model(dirIt->path().c_str());
			svms.push_back(tmp);
		}
	}
}

//---------------COMPUTING----------------
void ClassifierSVM::train(std::vector<Entry> entries){
	clearData();
	int descLen = entries.front().descriptor.cols;
	int numLabels = 0;
	numEntries = entries.size();
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

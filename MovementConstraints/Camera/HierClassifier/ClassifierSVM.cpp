/*
 * ClassifierSVM.cpp
 *
 *  Created on: 22-10-2013
 *      Author: jachu
 */

#include <cstring>

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

	svmParams.svm_type = C_SVC;
	svmParams.cache_size = 32;
	svmParams.eps = 0.001;
	svmParams.probability = 1;
}

void ClassifierSVM::clearData(){
	for(int i = 0; i < numEntries; i++){
		delete[] labData[i];
	}
	delete[] labData;

	delete[] dataLabels;

	delete[] svmProblem.W;
	/*delete[] weights;
	delete[] labels;*/

	svm_free_and_destroy_model(&svm);

	startup();
}

ClassifierSVM::ClassifierSVM() :
	Classifier(Classifier::SVM)
{
	startup();
}

ClassifierSVM::ClassifierSVM(const ClassifierSVM& old) :
	Classifier(Classifier::SVM)
{
	if(old.numEntries == 0){
		throw "Cannot copy ClassifierSVM loaded from cache";
	}
	clearData();

	numEntries = old.numEntries;
	numLabels = old.numLabels;
	descLen = old.descLen;
	cacheEnabled = old.cacheEnabled;
	svmParams = old.svmParams;

	labData = new svm_node*[numEntries];
	for(int e = 0; e < numEntries; e++){
		labData[e] = new svm_node[descLen];
		memcpy(labData[e], old.labData[e], (descLen + 1) * sizeof(svm_node));
	}

	svmProblem.l = old.svmProblem.l;
	svmProblem.y = dataLabels;
	svmProblem.x = labData;
	svmProblem.W = new double[numEntries];
	memcpy(svmProblem.W, old.svmProblem.W, numEntries * sizeof(double));

	dataLabels = new double[numEntries];
	memcpy(dataLabels, old.dataLabels, numEntries * sizeof(double));

	/*weights = new double[numLabels];
	memcpy(weights, old.weights, numLabels * sizeof(double));

	labels = new int[numLabels];
	memcpy(labels, old.labels, numLabels * sizeof(int));*/

	svm = svm_train(&svmProblem, &svmParams);
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

ClassifierSVM* ClassifierSVM::copy(){
	return new ClassifierSVM(*this);
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

	svmParams.svm_type = svmType;
	svmParams.kernel_type = kernelType;
	svmParams.gamma = gamma;
	svmParams.degree = degree;
}

void ClassifierSVM::saveCache(boost::filesystem::path file){
	svm_save_model(file.c_str(), svm);
}

void ClassifierSVM::loadCache(boost::filesystem::path file){
	svm = svm_load_model(file.c_str());
}

//---------------COMPUTING----------------
void ClassifierSVM::train(	const std::vector<Entry>& entries,
							const std::vector<double>& dataWeights)
{
	//map<int, int> mapLabels;
	clearData();
	svmProblem.l = numLabels;
	svmProblem.y = dataLabels;
	svmProblem.x = labData;
	svmProblem.W = new double[numEntries];

	descLen = entries.front().descriptor.cols;
	numEntries = entries.size();
	labData = new svm_node*[numEntries];
	dataLabels = new double[numEntries];
	numLabels = 0;
	for(int e = 0; e < numEntries; e++){
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
		dataLabels[e] = entries[e].label;
		numLabels = max(numLabels, entries[e].label);
		svmProblem.W[e] = dataWeights[e];
	}
	numLabels++;

	/*weights = new double[numLabels];
	labels = new int[numLabels];
	for(int l = 0; l < numLabels; l++){
		weights[l] = 0;
		labels[l] = l;
	}
	for(int e = 0; e < entries.size(); e++){
		weights[entries[e].label] += dataWeights[e];
	}*/

	svmParams.nr_weight = numLabels;
	/*svmParams.weight_label = labels;
	svmParams.weight = weights;*/

	if(svm_check_parameter(&svmProblem, &svmParams) != NULL){
		throw "Bad svm params";
	}
	svm = svm_train(&svmProblem, &svmParams);

}

cv::Mat ClassifierSVM::classify(cv::Mat features){
	Mat ret(1, numLabels, CV_32FC1);
	double* prob_est = new double[numLabels];
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

	svm_predict_probability(svm, data, prob_est);
	for(int l = 0; l < numLabels; l++){
		ret.at<float>(svm->label[l]) = prob_est[l];
	}

	delete[] data;
	delete[] prob_est;
	return ret;
}

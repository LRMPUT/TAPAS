/*
 * ClassifierSVM.cpp
 *
 *  Created on: 22-10-2013
 *      Author: jachu
 */

#include <cstring>
#include <cstdio>
#include <random>
#include <chrono>

//Boost
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include "Classifier.h"
#include "ClassifierSVM.h"


using namespace cv;
using namespace std;
using namespace boost;

#define INF (10e9)

void ClassifierSVM::startup(){
	numEntries = 0;

	labData = 0;
	dataLabels = 0;
	svmProblem.W = 0;
	svm = 0;
	scalesDiv = 0;
	scalesSub = 0;

	svmParams.svm_type = C_SVC;
	svmParams.cache_size = 32;
	svmParams.eps = 0.001;
	svmParams.probability = 1;
	svmParams.shrinking = 0;
}

void ClassifierSVM::clearData(){
	//cout << "numEntries = " << numEntries << endl;
	for(int i = 0; i < numEntries; i++){
		delete[] labData[i];
	}
	//cout << "Deleteing labData" << endl;
	delete[] labData;

	//cout << "Deleteing dataLabels" << endl;
	delete[] dataLabels;

	//cout << "Deleteing svmProblem.W" << endl;
	delete[] svmProblem.W;
	/*delete[] weights;
	delete[] labels;*/

	delete[] scalesSub;
	delete[] scalesDiv;

	//cout << "Destroying svm" << endl;
	svm_free_and_destroy_model(&svm);

	startup();
}


void ClassifierSVM::prepareProblem(	const std::vector<Entry>& entries,
									const std::vector<double>& productWeights)
{
	clearData();

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	default_random_engine generator(seed);
	uniform_real_distribution<double> uniRealDist(0.0, 1.0);
	uniform_int_distribution<int> uniIntDist(0, entries.size() - 1);

	descLen = entries.front().descriptor.cols;
	numEntries = entries.size();
	scalesSub = new double[descLen];
	scalesDiv = new double[descLen];

	//normalization
	/*double* meanVal = new double[descLen];
	double* varVal = new double[descLen];
	for(int i = 0; i < descLen; i++){
		meanVal[i] = 0;
		varVal[i] = 0;
	}

	for(int e = 0; e < numEntries; e++){
		double w = entries[e].weight;
		if(!productWeights.empty()){
			w *= productWeights[e];
		}
		for(int i = 0; i < descLen; i++){
			meanVal[i] += entries[e].descriptor.at<float>(i) * w;
		}
	}
	for(int e = 0; e < numEntries; e++){
		for(int i = 0; i < descLen; i++){
			double w = entries[e].weight;
			if(!productWeights.empty()){
				w *= productWeights[e];
			}
			varVal[i] += (entries[e].descriptor.at<float>(i) - meanVal[i])*
						(entries[e].descriptor.at<float>(i) - meanVal[i]) * w;
		}
	}

	for(int i = 0; i < descLen; i++){
		scalesSub[i] = meanVal[i];
		scalesDiv[i] = sqrt(varVal[i]);
	}
	delete[] meanVal;
	delete[] varVal;*/

	//boostrap
	vector<int> finalIdxs(numEntries, 0);
	{
		int idx = uniIntDist(generator);
		double beta = 0;
		double maxW = 0;
		for(int e = 0; e < numEntries; e++){
			maxW = max(maxW, entries[e].weight);
		}
		for(int e = 0; e < numEntries; e++){
			double w = entries[idx].weight;
			if(!productWeights.empty()){
				w *= productWeights[idx];
			}
			beta += 2*maxW*uniRealDist(generator);
			while(w < beta){
				beta -= w;
				idx = (idx + 1) % numEntries;
				w = entries[idx].weight;
				if(!productWeights.empty()){
					w *= productWeights[idx];
				}
			}
			finalIdxs[e] = idx;
		}
	}

	labData = new svm_node*[numEntries];
	dataLabels = new double[numEntries];
	numLabels = 0;
	svmProblem.W = new double[numEntries];
	for(int e = 0; e < numEntries; e++){
		labData[e] = new svm_node[descLen + 1];
		for(int i = 0; i < descLen; i++){
			svm_node tmp;
			tmp.index = i;
			tmp.value = entries[finalIdxs[e]].descriptor.at<float>(i);
			labData[e][i] = tmp;
		}
		svm_node tmp;
		tmp.index = -1;
		labData[e][descLen] = tmp;
		dataLabels[e] = entries[finalIdxs[e]].label;
		numLabels = max(numLabels, entries[finalIdxs[e]].label);
		svmProblem.W[e] = 1.0/numEntries;
		//svmProblem.W[e] = entries[finalIdxs[e]].weight;
		//if(!productWeights.empty()){
		//	svmProblem.W[e] *= productWeights[e];
		//}
	}
	numLabels++;

	vector<double> popWeights(numLabels, 0);
	vector<double> popBootstrap(numLabels, 0);
	for(int e = 0; e < numEntries; e++){
		double w = entries[e].weight;
		if(!productWeights.empty()){
			w *= productWeights[e];
		}
		popWeights[entries[e].label] += w;
		popBootstrap[dataLabels[e]]++;
	}
	for(int l = 0; l < numLabels; l++){
		cout << "label " << l << ", weight = " << popWeights[l] << ", boostrap = " << popBootstrap[l]/numEntries << endl;
	}

	//normalization
	double* maxVal = new double[descLen];
	double* minVal = new double[descLen];
	for(int i = 0; i < descLen; i++){
		maxVal[i] = -INF;
		minVal[i] = INF;
	}
	for(int e = 0; e < numEntries; e++){
		for(int i = 0; i < descLen; i++){
			maxVal[i] = max(maxVal[i], (double)labData[e][i].value);
			minVal[i] = min(minVal[i], (double)labData[e][i].value);
		}
	}
	for(int i = 0; i < descLen; i++){
		scalesSub[i] = minVal[i];
		scalesDiv[i] = maxVal[i] - minVal[i];
	}
	delete[] minVal;
	delete[] maxVal;

	/*double* meanVal = new double[descLen];
	double* varVal = new double[descLen];
	for(int i = 0; i < descLen; i++){
		meanVal[i] = 0;
		varVal[i] = 0;
	}

	for(int e = 0; e < numEntries; e++){
		for(int i = 0; i < descLen; i++){
			meanVal[i] += labData[e][i].value;
		}
	}
	for(int i = 0; i < descLen; i++){
		meanVal[i] /= numEntries;
	}
	for(int e = 0; e < numEntries; e++){
		for(int i = 0; i < descLen; i++){
			varVal[i] += (labData[e][i].value - meanVal[i])*
						(labData[e][i].value - meanVal[i]);
		}
	}
	for(int i = 0; i < descLen; i++){
		varVal[i] /= (numEntries - 1);
	}

	for(int i = 0; i < descLen; i++){
		scalesSub[i] = meanVal[i];
		scalesDiv[i] = sqrt(varVal[i]);
	}
	delete[] meanVal;
	delete[] varVal;*/

	for(int e = 0; e < numEntries; e++){
		for(int i = 0; i < descLen; i++){
			labData[e][i].value -= scalesSub[i];
			if(scalesDiv[i] != 0){
				labData[e][i].value /= scalesDiv[i];
			}
		}
	}

	ofstream logFile("dataNorm.log");
	for(int e = 0; e < numEntries; e++){
		logFile << dataLabels[e] << " ";
		for(int i = 0; i < descLen; i++){
			logFile << (i + 1) << ":" << labData[e][i].value << " ";
		}
		logFile << descLen + 1 << ":-1" << endl;
	}
	logFile.close();

	svmParams.nr_weight = 0;
	svmProblem.l = numEntries;
	svmProblem.y = dataLabels;
	svmProblem.x = labData;

	const char* checkRes= svm_check_parameter(&svmProblem, &svmParams);
	if(checkRes != NULL){
		cout << checkRes << endl;
		throw "Bad svm params";
	}
}

ClassifierSVM::ClassifierSVM() :
	Classifier(Classifier::SVM)
{
	svmParams.C = 1;
	startup();
}

ClassifierSVM::ClassifierSVM(const ClassifierSVM& old) :
	Classifier(Classifier::SVM)
{
	cout << "Copy constructing ClassifierSVM" << endl;
	//startup();

	numEntries = old.numEntries;
	numLabels = old.numLabels;
	descLen = old.descLen;
	cacheEnabled = old.cacheEnabled;
	svmParams = old.svmParams;

	//cout << "numEntries = " << numEntries
	//		<< ", numLabels = " << numLabels
	//		<< ", descLen = " << descLen << endl;

	labData = new svm_node*[numEntries];
	for(int e = 0; e < numEntries; e++){
		labData[e] = new svm_node[descLen + 1];
		memcpy(labData[e], old.labData[e], (descLen + 1) * sizeof(svm_node));
	}

	dataLabels = new double[numEntries];
	memcpy(dataLabels, old.dataLabels, numEntries * sizeof(double));

	svmProblem.l = old.svmProblem.l;
	svmProblem.y = dataLabels;
	svmProblem.x = labData;
	svmProblem.W = new double[numEntries];
	memcpy(svmProblem.W, old.svmProblem.W, numEntries * sizeof(double));

	scalesSub = new double[descLen];
	memcpy(scalesSub, old.scalesSub, descLen * sizeof(double));
	scalesDiv = new double[descLen];
	memcpy(scalesDiv, old.scalesDiv, descLen * sizeof(double));

	/*weights = new double[numLabels];
	memcpy(weights, old.weights, numLabels * sizeof(double));

	labels = new int[numLabels];
	memcpy(labels, old.labels, numLabels * sizeof(int));*/

	/*cout << "svmProblem.l = " << svmProblem.l << endl;
	cout << "svmProblem.y = {";
	for(int e = 0; e < numEntries; e++){
		cout << svmProblem.y[e] << ", ";
	}
	cout << "}" << endl;
	for(int e = 0; e < numEntries; e++){
		cout << "svmProblem.x[" << e << "] = {";
		for(int d = 0; d < descLen + 1; d++){
			cout << svmProblem.x[e][d].index << "(" << svmProblem.x[e][d].value << "), ";
		}
		cout << endl;
	}
	cout << "}" << endl;*/

	//waitKey();
	if(svm_check_parameter(&svmProblem, &svmParams) != 0){
		cout << svm_check_parameter(&svmProblem, &svmParams) << endl;
		throw "Bad svm params during ClassifierSVM copy constructing";
	}
	svm = svm_train(&svmProblem, &svmParams);
	cout << "End copy constructing ClassifierSVM" << endl;
}

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

/*static const unsigned int NUM_INDENTS_PER_SPACE=2;

const char * getIndent( unsigned int numIndents )
{
	static const char * pINDENT="                                      + ";
	static const unsigned int LENGTH=strlen( pINDENT );
	unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
	if ( n > LENGTH ) n = LENGTH;

	return &pINDENT[ LENGTH-n ];
}

// same as getIndent but no "+" at the end
const char * getIndentAlt( unsigned int numIndents )
{
	static const char * pINDENT="                                        ";
	static const unsigned int LENGTH=strlen( pINDENT );
	unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
	if ( n > LENGTH ) n = LENGTH;

	return &pINDENT[ LENGTH-n ];
}

int dump_attribs_to_stdout(TiXmlElement* pElement, unsigned int indent)
{
	if ( !pElement ) return 0;

	TiXmlAttribute* pAttrib=pElement->FirstAttribute();
	int i=0;
	int ival;
	double dval;
	const char* pIndent=getIndent(indent);
	printf("\n");
	while (pAttrib)
	{
		printf( "%s%s: value=[%s]", pIndent, pAttrib->Name(), pAttrib->Value());

		if (pAttrib->QueryIntValue(&ival)==TIXML_SUCCESS)    printf( " int=%d", ival);
		if (pAttrib->QueryDoubleValue(&dval)==TIXML_SUCCESS) printf( " d=%1.1f", dval);
		printf( "\n" );
		i++;
		pAttrib=pAttrib->Next();
	}
	return i;
}

void dump_to_stdout( TiXmlNode* pParent, unsigned int indent = 0 )
{
	if ( !pParent ) return;

	TiXmlNode* pChild;
	TiXmlText* pText;
	int t = pParent->Type();
	printf( "%s", getIndent(indent));
	int num;

	switch ( t )
	{
	case TiXmlNode::TINYXML_DOCUMENT:
		printf( "Document" );
		break;

	case TiXmlNode::TINYXML_ELEMENT:
		printf( "Element [%s]", pParent->Value() );
		num=dump_attribs_to_stdout(pParent->ToElement(), indent+1);
		switch(num)
		{
			case 0:  printf( " (No attributes)"); break;
			case 1:  printf( "%s1 attribute", getIndentAlt(indent)); break;
			default: printf( "%s%d attributes", getIndentAlt(indent), num); break;
		}
		break;

	case TiXmlNode::TINYXML_COMMENT:
		printf( "Comment: [%s]", pParent->Value());
		break;

	case TiXmlNode::TINYXML_UNKNOWN:
		printf( "Unknown" );
		break;

	case TiXmlNode::TINYXML_TEXT:
		pText = pParent->ToText();
		printf( "Text: [%s]", pText->Value() );
		break;

	case TiXmlNode::TINYXML_DECLARATION:
		printf( "Declaration" );
		break;
	default:
		break;
	}
	printf( "\n" );
	for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
	{
		dump_to_stdout( pChild, indent+1 );
	}
}*/

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

	double C = 1;
	svmPtr = pPtr->FirstChildElement("C");
	if(!svmPtr){
		throw "Bad settings file - no C value";
	}
	svmPtr->QueryDoubleAttribute("value", &C);

	svmParams.svm_type = svmType;
	svmParams.kernel_type = kernelType;
	svmParams.gamma = gamma;
	svmParams.degree = degree;
	svmParams.C = C;
}

void ClassifierSVM::saveCache(TiXmlElement* settings, boost::filesystem::path file){
	svm_save_model(file.c_str(), svm);

	TiXmlElement* pScales = new TiXmlElement("scales");
	settings->LinkEndChild(pScales);
	pScales->SetAttribute("desc_len", descLen);
	for(int d = 0; d < descLen; d++){
		TiXmlElement* pValue = new TiXmlElement("value");
		pScales->LinkEndChild(pValue);
		pValue->SetAttribute("idx", d);
		pValue->SetDoubleAttribute("sub", scalesSub[d]);
		pValue->SetDoubleAttribute("div", scalesDiv[d]);
	}

	TiXmlElement* pLabels = new TiXmlElement("labels");
	settings->LinkEndChild(pLabels);
	pLabels->SetAttribute("num", numLabels);
}

void ClassifierSVM::loadCache(TiXmlElement* settings, boost::filesystem::path file){
	clearData();

	svm = svm_load_model(file.c_str());

	TiXmlElement* pScales = settings->FirstChildElement("scales");
	if(!pScales){
		throw "Bad cache file - no scales";
	}
	pScales->QueryIntAttribute("desc_len", &descLen);
	scalesSub = new double[descLen];
	scalesDiv = new double[descLen];
	TiXmlElement* pValue = pScales->FirstChildElement("value");
	while(pValue){
		int d = -1;
		pValue->QueryIntAttribute("idx", &d);
		if(d == -1){
			throw "Bad cache file - no idx for value";
		}
		pValue->QueryDoubleAttribute("sub", &scalesSub[d]);
		pValue->QueryDoubleAttribute("div", &scalesDiv[d]);

		pValue = pValue->NextSiblingElement();
	}

	TiXmlElement* pLabels = settings->FirstChildElement("labels");
	if(!pLabels){
		throw "Bad cache file - no labels";
	}
	pLabels->QueryIntAttribute("num", &numLabels);
}

//---------------COMPUTING----------------
void ClassifierSVM::train(	const std::vector<Entry>& entries,
							const std::vector<double>& productWeights)
{
	cout << "ClassifierSVM::train()" << endl;

	prepareProblem(entries, productWeights);

	svm = svm_train(&svmProblem, &svmParams);

	cout << "End ClassifierSVM::train()" << endl;
}

cv::Mat ClassifierSVM::classify(cv::Mat features){
	Mat ret(1, numLabels, CV_32FC1);
	double* prob_est = new double[numLabels];
	svm_node* data = new svm_node[features.cols + 1];
	for(int i = 0; i < features.cols; i++){
		svm_node tmp;
		tmp.index = i;
		tmp.value = features.at<float>(i) - scalesSub[i];
		if(scalesDiv[i] != 0){
			tmp.value /= scalesDiv[i];
		}
		data[i] = tmp;
	}
	svm_node tmp;
	tmp.index = -1;
	data[features.cols] = tmp;

	if(svm_check_probability_model(svm) == 0){
		throw "Bad probability model";
	}
	svm_predict_probability(svm, data, prob_est);
	for(int l = 0; l < numLabels; l++){
		ret.at<float>(svm->label[l]) = prob_est[l];
	}

	delete[] data;
	delete[] prob_est;
	return ret;
}

void ClassifierSVM::crossValidate(const std::vector<Entry>& entries)
{
	double gridCMin = pow(2, -5), gridCMax = pow(2, 15), gridCStep = 3;
	double gridGMin = pow(2, -15), gridGMax = pow(2, 3), gridGStep = 3;
	prepareProblem(entries);
	double* results = new double[entries.size()];
	double bestC, bestG, bestScore = -1;
	for(double paramC = gridCMin; paramC <= gridCMax; paramC *= gridCStep){
		for(double paramG = gridGMin; paramG <= gridGMax; paramG *= gridGStep){
			cout << "Validating params: C = " << paramC << ", gamma = " << paramG << endl;
			svmParams.C = paramC;
			svmParams.gamma = paramG;
			svm_cross_validation(&svmProblem, &svmParams, 5, results);
			double score = 0;
			for(int e = 0; e < entries.size(); e++){
				//cout << results[e] << ", " << entries[e].label << ", weight = " << svmProblem.W[e] <<  endl;
				score += (abs(results[e] - entries[e].label) > 0.01 ? 0 : 1)*entries[e].weight;
			}
			cout << "Score = " << score << endl;
			if(score > bestScore){
				bestScore = score;
				bestC = paramC;
				bestG = paramG;
			}
		}
	}

	svmParams.C = bestC;
	svmParams.gamma = bestG;
	cout << "Best parameters: C = " << bestC << ", gamma = " << bestG << ", bestScore = " << bestScore << endl;

	delete[] results;
}

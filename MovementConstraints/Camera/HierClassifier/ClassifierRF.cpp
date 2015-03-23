/*
 * ClassifierRF.cpp
 *
 *  Created on: 12 mar 2015
 *      Author: jachu
 */

#include <chrono>

#include "ClassifierRF.h"

using namespace std;
using namespace cv;

#define INF (1e9)

/** \brief Funkcja inicjująca klasyfikator.
 *
 */
void ClassifierRF::startup()
{
	numEntries = 0;

	scalesDiv = 0;
	scalesSub = 0;

	params = CvRTParams(2,
						10,
						0,
						false,
						15,
						0,
						true,
						0,
						25,
						0.01f,
						CV_TERMCRIT_ITER);
}

/** \brief Funkcja czyszcząca dane klasyfikatora.
 *
 */
void ClassifierRF::clearData()
{
	trainData = Mat();
	dataLabels = Mat();

	delete[] scalesSub;
	delete[] scalesDiv;

	startup();
}

/** \brief Funkcja przygotowująca opis problemu dla CvRTrees.
 *
 */
void ClassifierRF::prepareProblem(const std::vector<Entry>& entries,
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

	trainData = Mat(numEntries, descLen, CV_32FC1);
	dataLabels = Mat(numEntries, 1, CV_32SC1);
	numLabels = 0;
	for(int e = 0; e < numEntries; e++){
		entries[finalIdxs[e]].descriptor.copyTo(trainData.rowRange(e, e + 1));
		dataLabels.at<int>(e) = entries[finalIdxs[e]].label;
		numLabels = max(numLabels, entries[finalIdxs[e]].label);
	}
	numLabels++;

//	ofstream logFileTrainData("data.log");
//	logFileTrainData << "trainData = " << trainData << endl;
//	logFileTrainData.close();

	vector<double> popWeights(numLabels, 0);
	vector<double> popBootstrap(numLabels, 0);
	for(int e = 0; e < numEntries; e++){
		double w = entries[e].weight;
		if(!productWeights.empty()){
			w *= productWeights[e];
		}
		popWeights[entries[e].label] += w;
		popBootstrap[dataLabels.at<int>(e)]++;
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
			maxVal[i] = max(maxVal[i], (double)((float)trainData.at<float>(e, i)));
			minVal[i] = min(minVal[i], (double)((float)trainData.at<float>(e, i)));
		}
	}
	for(int i = 0; i < descLen; i++){
		scalesSub[i] = minVal[i];
		scalesDiv[i] = maxVal[i] - minVal[i];
	}
	delete[] minVal;
	delete[] maxVal;

//	for(int i = 0; i < descLen; i++){
//		cout << "scalesSub[" << i << "] = " << scalesSub[i] <<
//				", scalesDiv[" << i << "] = " << scalesDiv[i] << endl;
//	}

	for(int e = 0; e < numEntries; e++){
		for(int i = 0; i < descLen; i++){
			trainData.at<float>(e, i) -= scalesSub[i];
			if(scalesDiv[i] != 0){
				trainData.at<float>(e, i) /= scalesDiv[i];
			}
		}
	}

//	ofstream logFileTrainDataNorm("dataNorm.log");
//	logFileTrainDataNorm << "trainData = " << trainData << endl;
//	logFileTrainDataNorm.close();
}

/** \brief Standardowy konstruktor.
 *
 */
ClassifierRF::ClassifierRF() :
	Classifier(Classifier::RF)
{
	startup();
}

/** \brief Konstruktor kopiujący.
 *
 */
ClassifierRF::ClassifierRF(const ClassifierRF& old) :
		Classifier(Classifier::RF)
{
	startup();

	numEntries = old.numEntries;
	numLabels = old.numLabels;
	descLen = old.descLen;
	cacheEnabled = old.cacheEnabled;
	params = old.params;

	scalesSub = new double[descLen];
	memcpy(scalesSub, old.scalesSub, descLen * sizeof(double));
	scalesDiv = new double[descLen];
	memcpy(scalesDiv, old.scalesDiv, descLen * sizeof(double));

	rf = old.rf;
}

/** \brief Konstruktor ładujący ustawienia z pliku XML.

*/
ClassifierRF::ClassifierRF(TiXmlElement* settings) :
		Classifier(Classifier::RF, settings)
{
	startup();
	loadSettings(settings);
}

ClassifierRF::~ClassifierRF()
{
	delete[] scalesSub;
	delete[] scalesDiv;
}

/** \brief Funkcja tworząca kopię klasyfikatora i zwracająca do niej wskaźnik.
 *
 */
ClassifierRF* ClassifierRF::copy()
{
	return new ClassifierRF(*this);
}

/** \brief Funkcja ładująca ustawienia z pliku XML.

*/
void ClassifierRF::loadSettings(TiXmlElement* settings)
{

}

/** \brief Funkcja zapisująca cache do pliku.
 *
 */
void ClassifierRF::saveCache(TiXmlElement* settings, boost::filesystem::path file)
{

}

/** \brief Funkcja ładująca cache do pliku.
 *
 */
void ClassifierRF::loadCache(TiXmlElement* settings, boost::filesystem::path file)
{
	throw "ClassifierRF::loadCache not supported";
}

//---------------COMPUTING----------------

/** \brief Funkcja ucząca klasyfikator z możliwością przekazania zewnętrznych wag.
 *
 */
void ClassifierRF::train(	const std::vector<Entry>& entries,
					const std::vector<double>& productWeights)
{
	prepareProblem(entries, productWeights);

	Mat varType(descLen + 1, 1, CV_8UC1, Scalar(CV_VAR_NUMERICAL));
	varType.at<unsigned char>(descLen) = CV_VAR_CATEGORICAL;

	rf.train(trainData,
			CV_ROW_SAMPLE,
			dataLabels,
			Mat(),
			Mat(),
			Mat()/*varType*/,
			Mat(),
			params);
}

/** \brief Funkcja klasyfikująca.
 * @return Macierz 1xliczba_etykiet z prawdopodobieństwem dla każdej etykiety.
 */
cv::Mat ClassifierRF::classify(cv::Mat features)
{
	Mat ret(1, numLabels, CV_32FC1, Scalar(0));

	Mat featNorm = features.clone();
	for(int d = 0; d < descLen; d++){
		featNorm.at<float>(d) -= scalesSub[d];
		if(scalesDiv[d] != 0){
			featNorm.at<float>(d) /= scalesDiv[d];
		}
	}

	float res = rf.predict_prob(featNorm);

//	cout << "res = " << res << endl;

	ret.at<float>(0) = 1 -res;
	ret.at<float>(1) = res;

	return ret;
}

/** \brief Funkcja dokonująca cross validation.
 *
 */
void ClassifierRF::crossValidate(const std::vector<Entry>& entries)
{
	throw "ClassifierRF::crossValidate not supported";
}



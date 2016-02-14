/*
 * ClassifierRF.h
 *
 *  Created on: 12 mar 2015
 *      Author: jachu
 */

#ifndef MOVEMENTCONSTRAINTS_CAMERA_HIERCLASSIFIER_CLASSIFIERRF_H_
#define MOVEMENTCONSTRAINTS_CAMERA_HIERCLASSIFIER_CLASSIFIERRF_H_

//STL
#include <vector>
//OpenCV
#include <opencv2/opencv.hpp>
//TinyXML
#include <tinyxml.h>
//Boost
#include <boost/filesystem.hpp>

#include "HierClassifier.h"

/** \brief Klasa będąca realizacją słabego klasyfikatora jako klasyfikator Random Forest.
 *
 */
class ClassifierRF : public Classifier {

//	CvRTParams params;
//	cv::ml:: params;
	cv::Ptr<cv::ml::RTrees> rf;

	cv::Mat trainData;
	cv::Mat dataLabels;

	/** Współczynnik skalowania dla danych dla każdego wymiaru wektora cech. */
	double* scalesSub;
	/** Współczynnik skalowania dla danych dla każdego wymiaru wektora cech. */
	double* scalesDiv;

	/** Liczba przykładów uczących. */
	int numEntries;
	/** Liczba wykorzystywanych etykiet. Etykiety są numerowane kolejnymi
	 * 	liczbami całkowitymi, zaczynając od 0.
	 */
	int numLabels;
	/** Długość wektora cech. */
	int descLen;
	/** Flaga informująca czy wczytywać i zapisywać cache. */
	bool cacheEnabled;

	/** \brief Funkcja inicjująca klasyfikator.
	 *
	 */
	void startup();

	/** \brief Funkcja czyszcząca dane klasyfikatora.
	 *
	 */
	void clearData();

	/** \brief Funkcja przygotowująca opis problemu dla biblioteki LibSVM.
	 *
	 */
	void prepareProblem(const std::vector<Entry>& entries,
						const std::vector<double>& productWeights = std::vector<double>());

public:
//---------------MISCELLANEOUS----------------

	/** \brief Standardowy konstruktor.
	 *
	 */
	ClassifierRF();

	/** \brief Konstruktor kopiujący.
	 *
	 */
	ClassifierRF(const ClassifierRF& old);

	/** \brief Konstruktor ładujący ustawienia z pliku XML.

	*/
	ClassifierRF(TiXmlElement* settings);

	virtual ~ClassifierRF();

	/** \brief Funkcja tworząca kopię klasyfikatora i zwracająca do niej wskaźnik.
	 *
	 */
	virtual ClassifierRF* copy();

	/** \brief Funkcja ładująca ustawienia z pliku XML.

	*/
	virtual void loadSettings(TiXmlElement* settings);

	/** \brief Funkcja zapisująca cache do pliku.
	 *
	 */
	virtual void saveCache(TiXmlElement* settings, boost::filesystem::path file);

	/** \brief Funkcja ładująca cache do pliku.
	 *
	 */
	virtual void loadCache(TiXmlElement* settings, boost::filesystem::path file);

//---------------COMPUTING----------------

	/** \brief Funkcja ucząca klasyfikator z możliwością przekazania zewnętrznych wag.
	 *
	 */
	virtual void train(	const std::vector<Entry>& entries,
						const std::vector<double>& productWeights = std::vector<double>());

	/** \brief Funkcja klasyfikująca.
	 * @return Macierz 1xliczba_etykiet z prawdopodobieństwem dla każdej etykiety.
	 */
	virtual cv::Mat classify(cv::Mat features);

	/** \brief Funkcja dokonująca cross validation.
	 *
	 */
	virtual void crossValidate(const std::vector<Entry>& entries);

	virtual cv::Mat normalizeFeat(const cv::Mat features);
};



#endif /* MOVEMENTCONSTRAINTS_CAMERA_HIERCLASSIFIER_CLASSIFIERRF_H_ */

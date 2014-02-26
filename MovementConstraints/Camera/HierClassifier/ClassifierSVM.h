/*
 * ClassifierSVM.h
 *
 *  Created on: 22-10-2013
 *      Author: jachu
 */

#ifndef CLASSIFIERSVM_H_
#define CLASSIFIERSVM_H_

//STL
#include <vector>
//OpenCV
#include <opencv2/opencv.hpp>
//TinyXML
#include <tinyxml.h>
//Boost
#include <boost/filesystem.hpp>
//LibSVM-weights
#include "../LibSVM-weights/svm.h"

#include "HierClassifier.h"
#include "Classifier.h"

/** \brief Klasa będąca realizacją słabego klasyfikatora jako klasyfikator SVM.
 *
 */
class ClassifierSVM : public Classifier {
	/** Struktura LibSVM zawierająca model nauczonego klasyfikatora. */
	svm_model* svm;
	/** Struktura LibSVM opisująca dane uczące dla klasyfikatora. */
	svm_problem svmProblem;
	/** Struktura LibSVM z parametrami dla klasyfikatora. */
	svm_parameter svmParams;
	/** Struktura LibSVM z wektorami cech dla klasyfikatora. */
	svm_node** labData;
	/** Tablica etykiet dla danych uczących. */
	double* dataLabels;
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
	ClassifierSVM();

	/** \brief Konstruktor kopiujący - używać tylko dla klasyfikatorów nauczonych,
	 * 			nie dla tych z danymi wczytanymi z cache.
	 *
	 */
	ClassifierSVM(const ClassifierSVM& old);

	/** \brief Konstruktor ładujący ustawienia z pliku XML.

	*/
	ClassifierSVM(TiXmlElement* settings);

	/** \brief Funkcja tworząca kopię klasyfikatora i zwracająca do niej wskaźnik.
	 *
	 */
	virtual ClassifierSVM* copy();

	virtual ~ClassifierSVM();

	/** \brief Funkcja ładująca ustawienia z pliku XML.

	*/
	virtual void loadSettings(TiXmlElement* settings);

	/** \brief Funkcja zapisująca cache do pliku.
	 *
	 */
	virtual void saveCache(boost::filesystem::path file);

	/** \brief Funkcja ładująca cache do pliku.
	 *
	 */
	virtual void loadCache(boost::filesystem::path file);

//---------------COMPUTING----------------

	/** \brief Funkcja ucząca klasyfikator z możliwością przekazania zewnętrznych wag.
	 *
	 */
	virtual void train(const std::vector<Entry>& entries,
					   const std::vector<double>& productWeights = std::vector<double>());

	/** \brief Funkcja klasyfikująca.
	 * @return Macierz 1xliczba_etykiet z prawdopodobieństwem dla każdej etykiety.
	 */
	virtual cv::Mat classify(cv::Mat features);

	/** \brief Funkcja dokonująca cross validation.
	 *
	 */
	virtual void crossValidate(const std::vector<Entry>& entries);
};


#endif /* CLASSIFIERSVM_H_ */

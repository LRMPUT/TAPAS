/*
 * HierClassifier.h
 *
 *  Created on: 10-10-2013
 *      Author: jachu
 */

#ifndef HIERCLASSIFIER_H_
#define HIERCLASSIFIER_H_

/** \brief Struktura przechowująca informacje o segmentach.
 *
 */
struct Entry{
	/** Opis za pomocą wektora cech. */
	cv::Mat descriptor;
	/** Etykieta przypisana do segmentu. */
	int label;
	/** Identyfikator segmentu na obrazie. */
	int imageId;
	/** Waga segmentu do celów uczenia. */
	double weight;
	Entry() : label(-1) {}
	Entry(int ilabel, cv::Mat idescriptor, double iweight, int iimageId = 0) :
		label(ilabel),
		weight(iweight),
		imageId(iimageId)
	{
		idescriptor.copyTo(descriptor);
	}

};

//STL
#include <vector>
#include <fstream>
//OpenCV
#include <opencv2/opencv.hpp>
//TinyXML
#include <tinyxml.h>
//Boost
#include <boost/filesystem.hpp>
//Hierarchical Classifier
#include "Classifier.h"

/** \brief Klasa odpowiadajaca za łączenie słabych klasyfikatorów.
 *
 */
class HierClassifier {
	friend class Debug;

	/** \brief Struktura przechowująca informacje o wektorach cech używanych
	 * 			w słabych klasyfikatorach.
	 */
	struct WeakClassifierInfo{
		int descBeg, descEnd;	// <descBeg, descEnd)
		WeakClassifierInfo() {}
		WeakClassifierInfo(int idescBeg, int idescEnd) :
			descBeg(idescBeg),
			descEnd(idescEnd)
		{}
	};

	/** Informacje o wektorach cech dla zbioru dostępnych słabych klasyfikatorów. */
	std::vector<WeakClassifierInfo> weakClassInfo;

	/** Zbiór dostępnych słabych klasyfikatorów. */
	std::vector<Classifier*> weakClassifiersSet;

	/** Dane dla słabych klasyfikatorów. */
	std::vector<std::vector<Entry> > dataClassifiers;

	/** Liczba dostępnych słabych klasyfikatorów. */
	int numWeakClassifiers;

	/** Liczba iteracji algorytmu Adaboost. */
	int numIterations;

	/** Rozmiary poszczególnych składowych wektora cech. */
	int histHLen, histSLen, histVLen, covarHSVLen, meanHSVLen;
	int covarLaserLen, meanLaserLen, kurtLaserLen, histDLen, histILen;

	/** Wektor z położeniami początków poszczególnych składowych wektora cech. */
	std::vector<int> descBeg;

	/** Macierz 3x3 kamery z danymi o ogniskowych i punktach centralnych. */
	cv::Mat cameraMatrix;

	/** Zbiór wybranych słabych klasyfikatorów. */
	std::vector<Classifier*> classifiers;

	/** Informacje o wektorach cech dla wybranych słabych klasyfikatorów. */
	std::vector<WeakClassifierInfo> classifiersInfo;

	/** Wagi dla wybranych słabych klasyfikatorów. */
	std::vector<double> weights;

	/** Liczba wykorzystywanych etykiet. Etykiety są numerowane kolejnymi
	 * 	liczbami całkowitymi, zaczynając od 0.
	 */
	int numLabels;

	/** Parametr k segmentacji obrazu. */
	float kSegment;

	/** Paramentr minimalnego segmentu dla segmentacji obrazu. */
	int minSizeSegment;

	/** Flaga informująca czy wczytywać i zapisywać cache. */
	bool cacheEnabled;

	int debugLevel;

	//cv::Mat projectPointsTo3D(	cv::Mat disparity);

	//cv::Mat projectPointsTo2D(	cv::Mat _3dImage);

	/** \brief Funkcja przytowująca dane dla słabych klasyfikatorów.
	 *
	 */
	void prepareData(const std::vector<Entry>& data);

	/** \brief Funkcja usuwająca dane dla klasyfikatorów i usuwająca wybrane
	 * 			klasyfikatory.
	 */
	void clearData();

public:

//---------------MISCELLANEOUS----------------

	/** \brief Standardowy konstruktor.
	 *
	 */
	HierClassifier(cv::Mat icameraMatrix);

	/** \brief Konstruktor ładujący ustawienia z pliku XML.

	*/
	HierClassifier(cv::Mat icameraMatrix, TiXmlElement* settings);

	~HierClassifier();

	/** \brief Funkcja ładująca ustawienia z pliku XML.

	*/	
	void loadSettings(TiXmlElement* settings);

	/** \brief Funkcja zapisująca cache do pliku.
	 *
	 */
	void saveCache(boost::filesystem::path file);

	/** \brief Funkcja ładująca cache do pliku.
	 *
	 */
	void loadCache(boost::filesystem::path file);

	/** \brief Funkcja przypisująca id segmentu na obrazie ręcznie oznaczonym
	 * 			do id segmentu na obrazie oznaczonym automatycznie.
	 */
	std::map<int, int> assignManualId(cv::Mat autoSegments, cv::Mat manualSegments);

	/** \brief Funkcja produkująca obraz z pokolorowanym segmentami,
	 * 			używana do testów segmentacji.
	 *
	 */
	cv::Mat colorSegments(const cv::Mat segments);

//---------------COMPUTING----------------

	/** \brief Funkcja trenująca klasyfikator za pomocą algorytmu Adaboost.
	 *
	 */
	void train(const std::vector<Entry>& data,
				int inumLabels);

	/**	\brief Funkcja klasyfikująca podany obraz wraz z chmurą punktów.
	 *
	 * 	@param image Macierz o 3 kanałach z obrazem RGB.
	 * 	@param terrain Macierz 5xN ze współrzędnymi, odległością i wartością "intensity"
	 * 				dla wszystkich punktów.
	 * 	@param segmentation Opcjonalna zewnętrzna segmentacja obrazu.
	 *	@return Macierze o rozmiarach obrazu z prawdopodobieństwami dla wszystkich etykiet.
	*/
	std::vector<cv::Mat> classify(cv::Mat image,
								  cv::Mat terrain = cv::Mat(),
								  cv::Mat segmentation = cv::Mat(),
								  cv::Mat maskIgnore = cv::Mat(),
				  	  	  	  	  int entryWeightThreshold = 0);
	
	/** \brief Funkcja ekstrahująca wektor cech.

	*/
	std::vector<Entry> extractEntries(cv::Mat image,
										cv::Mat terrain,
										cv::Mat regionsOnImage,
										cv::Mat maskIgnore = cv::Mat(),
										int entryWeightThreshold = 0);

	std::vector<Entry> extractEntriesGPU(cv::Mat imageBGR,
										cv::Mat terrain,
										cv::Mat regionsOnImage);

	/** \brief Funkcja segmentująca obraz używając danych RGB.
	 * 	@return Macierz z id segmentu dla każdego piksela na obrazie.
	 */
	cv::Mat segmentImage(cv::Mat image, int kCurSegment = -1);

	/** \brief Funkcja przeprowadzająca cross validation klasyfikatorów typu SVM.
	 *
	 */
	void crossValidateSVMs(const std::vector<Entry>& entries);
};


#endif /* HIERCLASSIFIER_H_ */


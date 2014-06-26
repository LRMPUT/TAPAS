/*
 * Kinect.cpp
 *
 *  Created on: Jun 26, 2014
 *      Author: jachu
 */

#include <chrono>
#include <opencv2/opencv.hpp>
#include "Kinect.h"

using namespace std;
using namespace cv;

void Kinect::run(){

	while(runThread){

		std::unique_lock<std::mutex> lck(mtx);

		lck.unlock();
		//cout << "Number of zeros: " << count << endl;

		std::chrono::milliseconds duration(20);
		std::this_thread::sleep_for(duration);
	}
}

Kinect::Kinect() :
		runThread(false)
{

}

Kinect::~Kinect() {
	closePort();
}

void Kinect::openPort(std::string port){
	runThread = true;
	readingThread = std::thread(&Kinect::run, this);
}

void Kinect::closePort(){
	runThread = false;
	readingThread.join();
}

bool Kinect::isOpen(){

}

//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
const cv::Mat Kinect:: getData(){
	Mat ret;
	std::unique_lock<std::mutex> lck(mtx);

	lck.unlock();

	return ret;
}

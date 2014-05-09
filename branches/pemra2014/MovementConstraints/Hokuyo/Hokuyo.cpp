/*
 * Hokuyo.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

#include <chrono>
#include <urg_cpp/Urg_driver.h>
#include <opencv2/opencv.hpp>
#include "Hokuyo.h"

using namespace std;
using namespace cv;

void Hokuyo::run(){
	hokuyo.start_measurement(	qrk::Urg_driver::Distance_intensity,
								qrk::Urg_driver::Infinity_times,
								0);
	while(runThread){
		vector<long int> distance;
		vector<unsigned short> intensity;

		//cout << "is_open: " << hokuyo.is_open() << endl;
		//cout << "status: " << hokuyo.status() << endl;
		//1 measurement doesn't work with Distance_intensity
		hokuyo.get_distance_intensity(distance, intensity);
		//hokuyo.get_distance_intensity(distance, intensity);
		//cout << "distance.size() = " << distance.size() << ", intensity.size() = " << intensity.size() << endl;
		//int count = 0;
		std::unique_lock<std::mutex> lck(mtx);
		for(int i = 0; i < distance.size(); i++){
			double angle = hokuyo.index2rad(i);
			//cout << "Point " << i << " = " << distance[i] << ", " << intensity[i] << endl;
			//cout << "Point " << i << " = (" << data[i]*cos(angle) << ", " << data[i]*sin(angle) << ")" << endl;
			//if(distance[i] == 0){
			//	count++;
			//}
			curMeas.at<int>(0, i) = distance[i]*cos(angle);
			curMeas.at<int>(1, i) = distance[i]*sin(angle);
			curMeas.at<int>(2, i) = distance[i];
			curMeas.at<int>(3, i) = intensity[i];
		}
		lck.unlock();
		//cout << "Number of zeros: " << count << endl;

		std::chrono::milliseconds duration(20);
		std::this_thread::sleep_for(duration);
	}
	hokuyo.stop_measurement();
}

Hokuyo::Hokuyo() :
		curMeas(4, HOKUYO_SCANS, CV_32SC1),
		runThread(false)
{

}

Hokuyo::~Hokuyo() {
	closePort();
}

void Hokuyo::openPort(std::string port){
	cout << hokuyo.open(port.c_str(), qrk::Urg_driver::Default_baudrate, qrk::Urg_driver::Serial) << endl;

	cout << hokuyo.product_type() << endl;
	//cout << hokuyo.deg2index(-135) << " " << hokuyo.deg2index(135) << endl;
	hokuyo.set_scanning_parameter(hokuyo.deg2index(-135), hokuyo.deg2index(135));
	runThread = true;
	readingThread = std::thread(&Hokuyo::run, this);
}

void Hokuyo::closePort(){
	runThread = false;
	readingThread.join();
	cout << "Closing hokuyo" << endl;
	hokuyo.close();
}

bool Hokuyo::isOpen(){
	return hokuyo.is_open();
}

//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
const cv::Mat Hokuyo:: getData(){
	Mat ret(curMeas.rows, curMeas.cols, CV_32SC1);
	std::unique_lock<std::mutex> lck(mtx);
	curMeas.copyTo(ret);
	lck.unlock();

	return ret;
}

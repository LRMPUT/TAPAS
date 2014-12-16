/*Copyright (c) 2014, TAPAS Team (cybair [at] put.poznan.pl), Poznan University of Technology
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

#include <chrono>
#include <urg_cpp/Urg_driver.h>
#include <opencv2/opencv.hpp>
#include "Hokuyo.h"

using namespace std;
using namespace cv;

void Hokuyo::run(){
	try{
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

			curMeas = Mat(4, distance.size(), CV_32SC1);
			curTimestamp = std::chrono::high_resolution_clock::now();
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
			dataValid = true;
			lck.unlock();
			//cout << "Number of zeros: " << count << endl;

			std::chrono::milliseconds duration(20);
			std::this_thread::sleep_for(duration);
		}
		hokuyo.stop_measurement();
	}
	catch(char const* error){
		cout << "Char exception in Hokuyo: " << error << endl;
		exit(1);
	}
	catch(std::exception& e){
		cout << "Std exception in Hokuyo: " << e.what() << endl;
		exit(1);
	}
	catch(...){
		cout << "Unexpected exception in Hokuyo" << endl;
		exit(1);
	}
}

Hokuyo::Hokuyo() :
		runThread(false),
		dataValid(false)
{

}

Hokuyo::~Hokuyo() {
	closePort();
}

void Hokuyo::openPort(std::string port){
	cout << "Hokuyo open status: " << hokuyo.open(port.c_str(), qrk::Urg_driver::Default_baudrate, qrk::Urg_driver::Serial) << endl;
	if (hokuyo.is_open())
	{
		cout << "Hokuyo product type : "<<hokuyo.product_type() << endl;
		//cout << hokuyo.deg2index(-135) << " " << hokuyo.deg2index(135) << endl;
		hokuyo.set_scanning_parameter(hokuyo.deg2step(-100), hokuyo.deg2step(100));
		runThread = true;
		readingThread = std::thread(&Hokuyo::run, this);
	}
}

void Hokuyo::closePort(){
	dataValid = false;
	runThread = false;
	if(readingThread.joinable()){
		readingThread.join();
	}
	cout << "Closing hokuyo" << endl;
	hokuyo.close();
}

bool Hokuyo::isOpen(){
	return hokuyo.is_open();
}

bool Hokuyo::isDataValid(){
	return dataValid;
}

//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
cv::Mat Hokuyo:: getData(std::chrono::high_resolution_clock::time_point &timestamp){
	Mat ret(curMeas.rows, curMeas.cols, CV_32SC1);
	std::unique_lock<std::mutex> lck(mtx);
	curMeas.copyTo(ret);
	timestamp = curTimestamp;
	lck.unlock();

	return ret;
}

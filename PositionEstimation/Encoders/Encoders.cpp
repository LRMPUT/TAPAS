/*
	Copyright (c) 2014,	TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
	Poznan University of Technology
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
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "Encoders.h"
#include <boost/circular_buffer.hpp>

using namespace std;
using namespace cv;
using namespace trobot;

Encoders::Encoders() : runThread(false), curEnc(2, 1, CV_32SC1), dataValid(false) {

}

Encoders::Encoders(Robot* irobot) : runThread(false), curEnc(2, 1, CV_32SC1), robot(irobot), dataValid(false) {

}

/*Encoders::Encoders(const std::string& device, unsigned int baud){
	openPort(device, baud);
}*/

Encoders::~Encoders() {
	closePort();
}

void Encoders::run(){
	try{
		while(runThread){

			serialPort.startReadCount();
			serialPort.write("req");
			//cout << "Data sent" << endl;
			static boost::circular_buffer<char> data(50);
			int left, right;
			bool readEncoders = false;
			int count = 0;
			static const int countLimit = 60;
			while(!readEncoders && count < countLimit){
				boost::circular_buffer<char> newData = serialPort.getDataRead();
				//cout << "Data received" << endl;
				//cout << "newData:" << endl;
				for(int i = 0; i < newData.size(); i++){
					//cout << newData[i];
					data.push_back(newData[i]);
				}
				//cout << endl;
				/*cout << "data:" << endl;
				for(int i = 0; i < data.size(); i++){
					cout << data[i];
				}
				cout << endl;*/
				int posBeg = searchBufferR(data, "E ");
				int posEnd = searchBufferR(data, "_");
				//cout << "posBeg = " << posBeg << ", posEnd = " << posEnd << endl;
				static const int bufferLen = 40;
				char buffer[bufferLen];
				if(posBeg >= 0){
					for(int i = 0; i < posBeg; i++){
						data.pop_front();
					}
					if(posBeg < posEnd && posEnd >= 0){
						for(int i = posBeg; i < posEnd; i++){
							buffer[i - posBeg] = data.front();
							data.pop_front();
						}
						buffer[posEnd - posBeg] = 0;
						sscanf(buffer, "E %d %d", &left, &right);
						readEncoders = true;
					}
				}
				usleep(1000);
				count++;
			}

			std::unique_lock<std::mutex> lck(mtx);
			curTimestamp = std::chrono::high_resolution_clock::now();
			if(count == countLimit){
				curEnc.at<int>(0) = -1;
				curEnc.at<int>(1) = -1;
				//throw "No encoders data received";
			}
			else{
				curEnc.at<int>(0) = -left;	//rotates in an opposite direction
				curEnc.at<int>(1) = right;
				dataValid = true;
			}
			//std::cout<<"Circular buffer result : " << -left << " " << right << std::endl;
			lck.unlock();

			std::chrono::milliseconds duration(20);
			std::this_thread::sleep_for(duration);
		}
	}
	catch(char const* error){
		cout << "Char exception in Encoders: " << error << endl;
		exit(1);
	}
	catch(std::exception& e){
		cout << "Std exception in Encoders: " << e.what() << endl;
		exit(1);
	}
	catch(...){
		cout << "Unexpected exception in Encoders" << endl;
		exit(1);
	}
}

void Encoders::openPort(const std::string& device, unsigned int baud){
	cout << "Opening encoders on port: " << device << endl;
	serialPort.open(baud, device);
	runThread = true;
	readingThread = std::thread(&Encoders::run, this);
}

void Encoders::closePort(){
	runThread = false;
	if(readingThread.joinable()){
		readingThread.join();
	}
	serialPort.close();
}

bool Encoders::isPortOpen(){
	return serialPort.isActive();
}

bool Encoders::isDataValid(){
	return dataValid;
}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS

//CV_32SC1 2x1: left, right encoder
cv::Mat Encoders::getEncoders(std::chrono::high_resolution_clock::time_point &timestamp){
	//cout << "Encoders::getEncoders" << endl;
	Mat ret(2, 1, CV_32SC1);
	std::unique_lock<std::mutex> lck(mtx);
	curEnc.copyTo(ret);
	timestamp = curTimestamp;
	lck.unlock();
	return ret;
}

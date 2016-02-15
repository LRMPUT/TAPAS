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

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <unistd.h>

#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include "Robot/Robot.h"
#include "Debug/Debug.h"

using namespace std;
using namespace boost;
using namespace cv;

// Starting point for our robot program
// Testing comment
int main(int argc, char* argv[])
{
	try{
//		std::system("espeak \"I'm TAPAS\"");
		cout<<"Starting program" << endl;
		Robot robot("../settings.xml");
		cout << "Robot created" << endl;
		Debug debug(&robot);

		/*robot.openImu("/dev/robots/imu2");
		robot.openEncoders("/dev/robots/encoders");
		robot.openHokuyo("/dev/robots/hokuyo");
		robot.openGps("/dev/robots/gps");
		robot.openRobotsDrive("/dev/robots/driverLeft", "/dev/robots/driverRight");
		robot.openCamera(vector<string>(1, "/dev/video0"));*/
		std::vector<boost::filesystem::path> dirsTrain;
		std::vector<boost::filesystem::path> dirsTestList;

		dirsTrain.push_back("../MovementConstraints/Camera/database/przejazd6");
		dirsTrain.push_back("../MovementConstraints/Camera/database/przejazd7");
		dirsTrain.push_back("../MovementConstraints/Camera/database/przejazd9");
		dirsTrain.push_back("../MovementConstraints/Camera/database/przejazd10");

		dirsTestList.push_back("../MovementConstraints/Camera/database/przejazdNew1");
		dirsTestList.push_back("../MovementConstraints/Camera/database/przejazdNew2");
		dirsTestList.push_back("../MovementConstraints/Camera/database/przejazdNew3");
		dirsTestList.push_back("../MovementConstraints/Camera/database/przejazdNew4");
		dirsTestList.push_back("../MovementConstraints/Camera/database/przejazdNew5");
		dirsTestList.push_back("../MovementConstraints/Camera/database/przejazdNew6");
		dirsTestList.push_back("../MovementConstraints/Camera/database/przejazdNew7");
		dirsTestList.push_back("../MovementConstraints/Camera/database/przejazdNew8");
//		dirsTestList.push_back("../MovementConstraints/Camera/database/przejazd9");
//		dirsTestList.push_back("../MovementConstraints/Camera/database/przejazd10");
//		dirsTestList.push_back("../MovementConstraints/Camera/database/przejazd11");

		for(boost::filesystem::path dir : dirsTestList){
			ofstream resLog("log/res.log", ios::app);
			resLog << dir.string() << endl;
			resLog.close();

			std::vector<boost::filesystem::path> dirsTest{dir};
			debug.testClassification(dirsTrain, dirsTest);
		}
//		debug.testClassification(dirsTrain, dirsTrain);

		/*char a;
		while((waitKey(200) & 0xff) != 's')
		{
		}
		robot.startCompetition();
		cout << "Robot stated" << endl;*/

	}
	catch(char const* error){
		cout << "Char exception in main: " << error << endl;
	}
	catch(std::exception& e){
		cout << "Std exception in main: " << e.what() << endl;
	}
	catch(...){
		cout << "Unexpected exception in main" << endl;
	}
}


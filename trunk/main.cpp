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

		/*robot.openImu("/dev/robots/imu2");
		robot.openEncoders("/dev/robots/encoders");
		robot.openHokuyo("/dev/robots/hokuyo");
		robot.openGps("/dev/robots/gps");
		robot.openRobotsDrive("/dev/robots/driverLeft", "/dev/robots/driverRight");
		robot.openCamera(vector<string>(1, "/dev/video0"));*/

		char a;
		while((waitKey(200) & 0xff) != 's')
		{
		}
		//robot.startCompetition();
		cout << "Robot stated" << endl;

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


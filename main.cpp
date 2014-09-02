#include <iostream>
#include <stdio.h>
#include <fstream>
#include <unistd.h>

#include <boost/filesystem.hpp>

#include "Robot/Robot.h"
#include "Debug/Debug.h"

using namespace std;
using namespace boost;

// Starting point for our robot program
// Testing comment
int main()
{
	try{
		cout<<"Starting program" << endl;
		Robot robot("../settings.xml");
		cout << "Robot created" << endl;
		//Debug debug(&robot);
		//debug.testConstraints(filesystem::path("../MovementConstraints/Camera/database/train"),
		//						filesystem::path("../MovementConstraints/Camera/database/classify"));

		//debug.testEncoders();

		//robot.openImu2("/dev/ttyACM0");
		cout << "IMU has been opened!" << endl;

		char a;
		while(1)
		{
			cin >> a;
			if ( a == 'q')
				break;
		}

	}
	catch(char const* error){
		cout << error << endl;
	}
}


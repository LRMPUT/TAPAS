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
		Robot robot("../settings.xml");
		cout << "Robot created" << endl;
		Debug debug(&robot);
		//debug.testConstraints(filesystem::path("../MovementConstraints/Camera/database/train"),
		//						filesystem::path("../MovementConstraints/Camera/database/classify"));
		vector<filesystem::path> dirsLearn, dirsClassify;
		dirsLearn.push_back(filesystem::path("../MovementConstraints/Camera/database/learn/przejazd24"));
		dirsLearn.push_back(filesystem::path("../MovementConstraints/Camera/database/learn/przejazd22"));
		dirsClassify.push_back(filesystem::path("../MovementConstraints/Camera/database/learn/przejazd25"));
		dirsClassify.push_back(filesystem::path("../MovementConstraints/Camera/database/learn/przejazd14"));
		debug.testClassification(dirsLearn, dirsClassify);
		//debug.testEncoders();
		int a;
		//while(1)
		//{


		//}

	}
	catch(char const* error){
		cout << error << endl;
	}
}


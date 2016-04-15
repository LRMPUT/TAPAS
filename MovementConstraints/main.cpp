#include "MovementConstraints.h"
 
using namespace std;

int main(int argc, char *argv[]) {
	try{
		ros::init(argc, argv, "MovementConstraints");

		TiXmlDocument settings("settings.xml");
		if(!settings.LoadFile()){
	//		cout << "Settings file: " << settingsFile << endl;
			throw settings.ErrorDesc();
		}

		TiXmlElement *pMovementConstraints = settings.FirstChildElement("Robot")->
			FirstChildElement("MovementConstraints");
		MovementConstraints m(pMovementConstraints);
//		m.openHokuyo("/dev/robots/hokuyo");
		ros::spin();
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
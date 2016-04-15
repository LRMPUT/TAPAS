#include "Camera.h"
 
using namespace std;

int main(int argc, char *argv[]) {
	try{
		ros::init(argc, argv, "Camera");

		TiXmlDocument settings("settings.xml");
		if(!settings.LoadFile()){
	//		cout << "Settings file: " << settingsFile << endl;
			throw settings.ErrorDesc();
		}

		TiXmlElement *pCamera = settings.FirstChildElement("Robot")->
			FirstChildElement("MovementConstraints")->FirstChildElement("Camera");
		Camera c(pCamera);
		c.open(vector<string>(1, "/dev/video0"));
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
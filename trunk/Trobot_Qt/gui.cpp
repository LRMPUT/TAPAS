#include "trobotqt.h"
#include <QtGui/QApplication>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
	try{
		QApplication a(argc, argv);
		if(argc != 2){
			throw "Usage: gui settings_file";
		}
		TrobotQt w(argv[1]);
		w.show();
		return a.exec();
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

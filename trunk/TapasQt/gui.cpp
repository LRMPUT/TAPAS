/*
    TapasQt is a GUI for TAPAS library
    Copyright (C) 2014, TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
	Poznan University of Technology

    This library is free software; you can redistribute it and/or modify it under
    the terms of the GNU Lesser General Public License as published by the
    Free Software Foundation; either version 2.1 of the License, or (at your option)
    any later version.

    This library is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
    details.

    You should have received a copy of the GNU Lesser General Public License along
    with this library; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include "TapasQt.h"
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
		TapasQt w(argv[1]);
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

/*
    TapasQt is a GUI for TAPAS library
    Copyright (C) 2014, TAPAS Team (cybair [at] put.poznan.pl), Poznan University of Technology

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

#include <iostream>

#include "TapasQt.h"
#include "CameraWindow.h"

using namespace std;

CameraWindow::CameraWindow(Robot* irobot, Debug* idebug, QWidget *parent, Qt::WFlags flags)
	: robot(irobot),
	  debug(idebug),
	  QMainWindow(parent, flags)
{
	ui.setupUi(this);
}
CameraWindow::~CameraWindow(){

}

void CameraWindow::keyPressEvent(QKeyEvent* event){
	/*if(mainWindow->drive->isOpen() && event->isAutoRepeat() == false){
		if(mainWindow->drive->getState() == Nothing){
			switch(event->key()){
				case Qt::Key_W :{
					cout << "forward" << endl;
					mainWindow->drive->goForward();
					event->accept();
					break;
				}
				case Qt::Key_S :{
					cout << "backward" << endl;
					mainWindow->drive->goBackward();
					event->accept();
					break;
				}
				case Qt::Key_A :{
					cout << "left" << endl;
					mainWindow->drive->goLeft();
					event->accept();
					break;
				}
				case Qt::Key_D :{
					cout << "right" << endl;
					mainWindow->drive->goRight();
					event->accept();
					break;
				}
			}
		}
	}*/
}

void CameraWindow::keyReleaseEvent(QKeyEvent* event){
	/*if(mainWindow->drive->isOpen() && event->isAutoRepeat() == false){
		switch(event->key()){
			case Qt::Key_W :{
				if(mainWindow->drive->getState() == Forward){
					mainWindow->drive->stop();
					cout << "forward stop" << endl;
				}
				event->accept();
				break;
			}
			case Qt::Key_S :{
				if(mainWindow->drive->getState() == Backward){
					mainWindow->drive->stop();
					cout << "backward stop" << endl;
				}
				event->accept();
				break;
			}
			case Qt::Key_A :{
				if(mainWindow->drive->getState() == Left){
					mainWindow->drive->stop();
					cout << "left stop" << endl;
				}
				event->accept();
				break;
			}
			case Qt::Key_D :{
				if(mainWindow->drive->getState() == Right){
					mainWindow->drive->stop();
					cout << "right stop" << endl;
				}
				event->accept();
				break;
			}
		}
	}*/
}

void CameraWindow::setFrame(uchar* data, int width, int height){
	ui.cameraLabel->setPixmap(QPixmap::fromImage(QImage(data, width, height, QImage::Format_RGB888)));
	ui.cameraLabel->update();
}

#include "CameraWindow.h"
#include <iostream>

using namespace std;

CameraWindow::CameraWindow(TrobotQt* imainWindow, QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags), mainWindow(imainWindow)
{
	ui.setupUi(this);
}
CameraWindow::~CameraWindow(){

}

void CameraWindow::keyPressEvent(QKeyEvent* event){
	if(mainWindow->drive != NULL && event->isAutoRepeat() == false){
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
	}
}

void CameraWindow::keyReleaseEvent(QKeyEvent* event){
	if(mainWindow->drive != NULL && event->isAutoRepeat() == false){
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
	}
}

void CameraWindow::setFrame(uchar* data, int width, int height){
	ui.cameraLabel->setPixmap(QPixmap::fromImage(QImage(data, width, height, QImage::Format_RGB888)));
	ui.cameraLabel->update();
}

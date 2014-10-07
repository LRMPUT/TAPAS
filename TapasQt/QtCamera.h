/*
 * QtCamera.h
 *
 *  Created on: 12-11-2013
 *      Author: jachu
 */

#ifndef QTCAMERA_H_
#define QTCAMERA_H_

//Qt
#include <QtCore/QObject>
#include <QtCore/QTimer>
//OpenCV
#include <opencv2/opencv.hpp>
//Trobot Qt
#include "CameraWindow.h"
#include "../Robot/Robot.h"
#include "../Debug/Debug.h"
#include "ui_TapasQt.h"

class QtCamera : public QObject
{
	Q_OBJECT

	Ui::TapasQtClass* ui;
	Robot* robot;
	Debug* debug;

	CameraWindow* remoteCamera;

	QTimer refreshTimer;

public:
	QtCamera(Ui::TapasQtClass* iui, Robot* irobot, Debug* idebug);
	virtual ~QtCamera();

	std::vector<cv::Mat> getFrame();

public slots:
	void refresh();
	void connect();
	void disconnect();

	void openCameraWindow();
	void cameraWindowClosed();
};


#endif /* QTCAMERA_H_ */

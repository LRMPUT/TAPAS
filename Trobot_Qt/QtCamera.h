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

class QtCamera : public QObject
{
	Q_OBJECT

	Ui::TrobotQtClass* ui;
	Robot* robot;
	Debug* debug;

	QTimer refreshTimer;

public:
	QtCamera(Ui::TrobotQtClass* iui, Robot* irobot, Debug* idebug);
	virtual ~QtCamera();

	cv::Mat getFrame();

public slots:
	void refresh();
	void connect();
	void disconnect();
};


#endif /* QTCAMERA_H_ */

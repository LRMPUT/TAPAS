/*
 * QtHokuyo.h
 *
 *  Created on: 12-11-2013
 *      Author: jachu
 */

#ifndef QTHOKUYO_H_
#define QTHOKUYO_H_

//Qt
#include <QtCore/QObject>
#include <QtCore/QTimer>
//OpenCV
#include <opencv2/opencv.hpp>

class QtHokuyo : public QObject
{
	Q_OBJECT

	Ui::TrobotQtClass* ui;
	Robot* robot;
	Debug* debug;

	QTimer refreshTimer;

public:
	QtHokuyo(Ui::TrobotQtClass* iui, Robot* irobot, Debug* idebug);
	virtual ~QtHokuyo();

	cv::Mat getDistance();

public slots:
	void refresh();
	void connect();
	void disconnect();
};


#endif /* QTHOKUYO_H_ */

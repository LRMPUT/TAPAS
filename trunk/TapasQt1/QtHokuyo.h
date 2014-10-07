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
//RobotsIntellect
#include "../Robot/Robot.h"
#include "../Debug/Debug.h"
//Trobot Qt
#include "ui_TapasQt.h"


class QtHokuyo : public QObject
{
	Q_OBJECT

	Ui::TapasQtClass* ui;
	Robot* robot;
	Debug* debug;

	QPixmap map;
	QTimer refreshTimer;

public:
	QtHokuyo(Ui::TapasQtClass* iui, Robot* irobot, Debug* idebug);
	virtual ~QtHokuyo();

	cv::Mat getData();

public slots:
	void refresh();
	void connect();
	void disconnect();
};


#endif /* QTHOKUYO_H_ */

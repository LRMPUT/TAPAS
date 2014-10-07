/*
 * QtGps.h
 *
 *  Created on: 15-07-2013
 *      Author: jachu
 */

#ifndef QTGPS_H_
#define QTGPS_H_

//Qt
#include<QtCore/QObject>
#include<QtCore/QTimer>

#include "../Robot/Robot.h"
#include "../Debug/Debug.h"
#include "ui_TapasQt.h"

class QtGps : public QObject {
	Q_OBJECT

	Ui::TapasQtClass* ui;
	Robot* robot;
	Debug* debug;

	QTimer refreshTimer;

public:
	QtGps(Ui::TapasQtClass* iui, Robot* irobot, Debug* idebug);
	virtual ~QtGps();

public slots:
	void connect();
	void disconnect();
	void refresh();
	void setZeroPoint();
};


#endif /* QTGPS_H_ */

/*
 * QtGps.h
 *
 *  Created on: 15-07-2013
 *      Author: jachu
 */

#ifndef QTGPS_H_
#define QTGPS_H_

#include<QtCore/QObject>
#include "../Robot/Robot.h"
#include "ui_trobotqt.h"

class QtGps : public QObject {
	Q_OBJECT

	Ui::TrobotQtClass* ui;
	Robot* robot;

public:
	QtGps(Ui::TrobotQtClass* iui, Robot* irobot);
	virtual ~QtGps();

public slots:
	void connect();
	void disconnect();
};


#endif /* QTGPS_H_ */

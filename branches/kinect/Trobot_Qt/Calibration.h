/*
 * Calibration.h
 *
 *  Created on: 15-11-2013
 *      Author: jachu
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <QtCore/QObject>
//Robots Intellect
#include "../Debug/Debug.h"
//Trobot Qt
#include "ui_trobotqt.h"

class Calibration : public QObject
{
	Q_OBJECT

	int index;

	Ui::TrobotQtClass* ui;
	Debug* debug;
public:
	Calibration(Ui::TrobotQtClass* iui, Debug* idebug);
	virtual ~Calibration();
public slots:
	void getData();
	void reset();
};


#endif /* CALIBRATION_H_ */

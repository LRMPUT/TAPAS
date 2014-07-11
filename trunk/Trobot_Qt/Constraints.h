/*
 * Constraints.h
 *
 *  Created on: 09-07-2014
 *      Author: jachu
 */

#ifndef CONSTRAINTS_H_
#define CONSTRAINTS_H_


#include <QtCore/QObject>
#include <QtCore/QTimer>
//TAPAS
#include "../Debug/Debug.h"
//Trobot Qt
#include "ui_trobotqt.h"
#include "Viewer.h"

class Constraints : public QObject
{
	Q_OBJECT

	Ui::TrobotQtClass* ui;
	Debug* debug;

	Viewer* viewer;

	QTimer timer;

	void updateCameraView();
	void updateMapView();
public:
	Constraints(Ui::TrobotQtClass* iui, Debug* idebug);
	virtual ~Constraints();
public slots:
	void updateViews();
};



#endif /* CONSTRAINTS_H_ */

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
#include "ui_TapasQt.h"
#include "Viewer.h"

class Constraints : public QObject
{
	Q_OBJECT

	Ui::TapasQtClass* ui;
	Debug* debug;

	Viewer* viewer;

	QTimer timer;

	void updateCameraView();
	void updateMapView();
	void updateClassificationView();
	void updateGlobalPlanView();
public:
	Constraints(Ui::TapasQtClass* iui, Debug* idebug);
	virtual ~Constraints();
public slots:
	void updateViews();
};



#endif /* CONSTRAINTS_H_ */

/*
    TapasQt is a GUI for TAPAS library
    Copyright (C) 2014, TAPAS Team (cybair [at] put.poznan.pl), Poznan University of Technology

    This library is free software; you can redistribute it and/or modify it under
    the terms of the GNU Lesser General Public License as published by the
    Free Software Foundation; either version 2.1 of the License, or (at your option)
    any later version.

    This library is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
    details.

    You should have received a copy of the GNU Lesser General Public License along
    with this library; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

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

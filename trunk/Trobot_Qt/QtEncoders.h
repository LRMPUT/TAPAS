/*
 * QtEncoders.h
 *
 *  Created on: 21-03-2014
 *      Author: jachu
 */

#ifndef QTENCODERS_H_
#define QTENCODERS_H_

#include <QtCore/QObject>
#include <string>
#include "../Robot/Robot.h"
#include "../Debug/Debug.h"
#include "ui_trobotqt.h"

class QtEncoders : public QObject
{

	Q_OBJECT

	Robot* robot;
	Debug* debug;
public:
	QtEncoders(Ui::TrobotQtClass* iui, Robot* irobot, Debug* idebug);
	~QtEncoders();
	bool isOpen();
private:
	Ui::TrobotQtClass* ui;

public slots:

	void openEncoders();
	void closeEncoders();
};

#endif /* QTENCODERS_H_ */

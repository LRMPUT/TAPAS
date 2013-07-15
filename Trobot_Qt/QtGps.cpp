/*
 * QtGps.cpp
 *
 *  Created on: 15-07-2013
 *      Author: jachu
 */

#include "QtGps.h"

QtGps::QtGps(Ui::TrobotQtClass* iui, Robot* irobot) : ui(iui), robot(irobot) {
	QObject::connect(ui->gpsConnectButton, SIGNAL(clicked()), this, SLOT(connect()));
	QObject::connect(ui->gpsDisconnectButton, SIGNAL(clicked()), this, SLOT(disconnect()));
}

QtGps::~QtGps(){

}

void QtGps::connect(){
	if(ui->gpsPortCombo->count() != 0){
		robot->openGps(ui->gpsPortCombo->currentText().toAscii().data());
	}
}

void QtGps::disconnect(){
	robot->closeGps();
}



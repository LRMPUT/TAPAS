/*
 * QtEncoders.cpp
 *
 *  Created on: 21-03-2014
 *      Author: jachu
 */

#include "QtEncoders.h"

using namespace std;

QtEncoders::QtEncoders(Ui::TrobotQtClass* iui, Robot* irobot, Debug* idebug):
	ui(iui),
	robot(irobot),
	debug(idebug)
{
	QObject::connect(ui->encodersConnectButton, SIGNAL(clicked()), this, SLOT(openEncoders()));
	QObject::connect(ui->encodersDisconnectButton, SIGNAL(clicked()), this, SLOT(closeEncoders()));
}

QtEncoders::~QtEncoders(){

}

bool QtEncoders::isOpen(){
	return robot->isEncodersOpen();
}

void QtEncoders::openEncoders(){
	if(ui->encodersPortCombo->count() != 0){
		cout << "Opening encoders on port: " << ui->encodersPortCombo->currentText().toAscii().data() << endl;
		robot->openEncoders(string(ui->encodersPortCombo->currentText().toAscii().data()));
	}
}

void QtEncoders::closeEncoders(){
	robot->closeEncoders();
}

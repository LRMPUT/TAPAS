#include "trobotqt.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	TrobotQt w;
	w.show();
	return a.exec();
}

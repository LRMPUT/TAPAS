/********************************************************************************
** Form generated from reading UI file 'CameraWindow.ui'
**
** Created: Fri Oct 16 11:46:06 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CAMERAWINDOW_H
#define UI_CAMERAWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_cameraWindow
{
public:
    QWidget *centralwidget;
    QLabel *cameraLabel;

    void setupUi(QMainWindow *cameraWindow)
    {
        if (cameraWindow->objectName().isEmpty())
            cameraWindow->setObjectName(QString::fromUtf8("cameraWindow"));
        cameraWindow->resize(711, 555);
        centralwidget = new QWidget(cameraWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        cameraLabel = new QLabel(centralwidget);
        cameraLabel->setObjectName(QString::fromUtf8("cameraLabel"));
        cameraLabel->setGeometry(QRect(20, 20, 640, 480));
        cameraLabel->setFrameShape(QFrame::StyledPanel);
        cameraWindow->setCentralWidget(centralwidget);

        retranslateUi(cameraWindow);

        QMetaObject::connectSlotsByName(cameraWindow);
    } // setupUi

    void retranslateUi(QMainWindow *cameraWindow)
    {
        cameraWindow->setWindowTitle(QApplication::translate("cameraWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        cameraLabel->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class cameraWindow: public Ui_cameraWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CAMERAWINDOW_H

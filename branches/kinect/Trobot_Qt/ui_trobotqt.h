/********************************************************************************
** Form generated from reading UI file 'trobotqt.ui'
**
** Created: Thu Apr 24 23:35:04 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TROBOTQT_H
#define UI_TROBOTQT_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QScrollBar>
#include <QtGui/QSlider>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TrobotQtClass
{
public:
    QWidget *centralWidget;
    QTabWidget *mainTab;
    QWidget *cameraTab;
    QLabel *cameraLabel;
    QPushButton *cameraNewWindowButton;
    QWidget *imuTab;
    QLabel *imuDisplayLabel;
    QGroupBox *imuValuesGroupBox;
    QGroupBox *accelGroupBox;
    QWidget *gridLayoutWidget_7;
    QGridLayout *gridLayout_5;
    QCheckBox *accelZcheckBox;
    QLabel *label_12;
    QLineEdit *accelScaleLineEdit;
    QCheckBox *accelYcheckBox;
    QCheckBox *accelXcheckBox;
    QGroupBox *gyroGroupBox;
    QWidget *gridLayoutWidget_9;
    QGridLayout *gridLayout_7;
    QCheckBox *gyroXcheckBox;
    QCheckBox *gyroYcheckBox;
    QCheckBox *gyroZcheckBox;
    QLabel *label_15;
    QLineEdit *gyroScaleLineEdit;
    QGroupBox *magGroupBox;
    QWidget *gridLayoutWidget_8;
    QGridLayout *gridLayout_6;
    QCheckBox *magXcheckBox;
    QCheckBox *magYcheckBox;
    QCheckBox *magZcheckBox;
    QLabel *label_13;
    QLineEdit *magScaleLineEdit;
    QGroupBox *eulerGroupBox;
    QWidget *gridLayoutWidget_10;
    QGridLayout *gridLayout_8;
    QCheckBox *eulerRollCheckBox;
    QCheckBox *eulerPitchCheckBox;
    QCheckBox *eulerYawCheckBox;
    QLabel *label_16;
    QLineEdit *eulerScaleLineEdit;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QPushButton *imuClearButton;
    QPushButton *imuStartButton;
    QPushButton *imuStopButton;
    QWidget *gridLayoutWidget_6;
    QGridLayout *gridLayout_4;
    QLabel *label_10;
    QLineEdit *imuTimeScaleLineEdit;
    QLabel *label_11;
    QCheckBox *imuAutoScrollCheckBox;
    QScrollBar *imuTimeScrollBar;
    QScrollBar *imuValueScrollBar;
    QWidget *gpsTab;
    QLabel *gpsDisplayLabel;
    QGroupBox *gpsValuesGroupBox;
    QWidget *verticalLayoutWidget_8;
    QVBoxLayout *verticalLayout_8;
    QCheckBox *longtitudeCheckBox;
    QCheckBox *lattitudeCheckBox;
    QWidget *verticalLayoutWidget_9;
    QVBoxLayout *verticalLayout_9;
    QLabel *label_7;
    QLabel *gpsSatelitesLabel;
    QLabel *label_9;
    QLabel *gpsFixLabel;
    QLabel *label_18;
    QLabel *gpsXLabel;
    QLabel *gpsYLabel;
    QPushButton *gpsZeroPointButton;
    QWidget *sensorsTab;
    QGroupBox *sensorsAnalogGroupBox;
    QWidget *gridLayoutWidget_4;
    QGridLayout *sensorsAnalogLayout;
    QLabel *label_19;
    QLabel *label_20;
    QLabel *label_21;
    QGroupBox *sensorsDigitalGroupBox;
    QWidget *gridLayoutWidget_5;
    QGridLayout *sensorsDigitalLayout;
    QLabel *label_14;
    QLabel *label_31;
    QLabel *label_32;
    QWidget *robotDriveTab;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_2;
    QPushButton *robotDriveLeftButton;
    QPushButton *robotDriveDownButton;
    QPushButton *robotDriveRightButton;
    QPushButton *robotDriveUpButton;
    QGroupBox *robotDriveMotorCtrlGroupBox;
    QWidget *gridLayoutWidget_3;
    QGridLayout *gridLayout_3;
    QSlider *robotDriveRightMotorSlider;
    QLabel *label;
    QLabel *label_3;
    QLabel *robotDriveLeftMotorLabel;
    QLabel *robotDriveRightMotorLabel;
    QPushButton *robotDriveLeftMotorStopButton;
    QPushButton *robotDriveRightMotorStopButton;
    QSlider *robotDriveLeftMotorSlider;
    QGroupBox *groupBox;
    QScrollBar *robotDriveSteeringScrollBar;
    QScrollBar *robotDriveThrottleScrollBar;
    QLabel *robotDriveSteeringLabel;
    QLabel *robotDriveThrottleLabel;
    QPushButton *robotDriveThrottleStopButton;
    QWidget *hokuyoTab;
    QLabel *hokuyoDisplayLabel;
    QWidget *recordTab;
    QGroupBox *saRateGroupBox;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QCheckBox *includeHokuyoCheckBox;
    QLineEdit *saRateHokuyoLineEdit;
    QCheckBox *includeEncodersCheckBox;
    QLineEdit *saRateEncodersLineEdit;
    QCheckBox *includeGpsCheckBox;
    QLineEdit *saRateGpsLineEdit;
    QCheckBox *includeCamerasCheckBox;
    QLineEdit *saRateCamerasLineEdit;
    QCheckBox *includeImuCheckBox;
    QLineEdit *saRateImuLineEdit;
    QGroupBox *groupBox_2;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_17;
    QLineEdit *recPathLineEdit;
    QPushButton *startRecButton;
    QPushButton *pauseResumeRecButton;
    QPushButton *stopRecButon;
    QLabel *recStatusLabel;
    QWidget *DevicesTab;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout_11;
    QLabel *label_22;
    QPushButton *hokuyoConnectButton;
    QPushButton *hokuyoDisconnectButton;
    QLabel *label_8;
    QComboBox *hokuyoPortCombo;
    QLabel *hokuyoStatusLabel;
    QCheckBox *hokuyoAutoCheckBox;
    QVBoxLayout *verticalLayout_6;
    QLabel *label_23;
    QPushButton *robotDriveConnectButton;
    QPushButton *robotDriveDisconnectButton;
    QLabel *label_4;
    QComboBox *robotDriversLeftPortCombo;
    QLabel *label_35;
    QComboBox *robotDriversRightPortCombo;
    QLabel *robotDriveStatusLabel;
    QCheckBox *robotDriveAutoCheckBox;
    QVBoxLayout *verticalLayout_12;
    QLabel *label_30;
    QPushButton *encodersConnectButton;
    QPushButton *encodersDisconnectButton;
    QLabel *label_33;
    QComboBox *encodersPortCombo;
    QLabel *label_34;
    QCheckBox *encodersAutoCheckBox;
    QVBoxLayout *verticalLayout_10;
    QLabel *label_24;
    QPushButton *sensorsConnectButton;
    QPushButton *sensorsDisconnectButton;
    QLabel *label_6;
    QComboBox *sensorsPortCombo;
    QLabel *sensorsStatusLabel;
    QCheckBox *sensorsAutoCheckBox;
    QVBoxLayout *verticalLayout_7;
    QLabel *label_25;
    QPushButton *gpsConnectButton;
    QPushButton *gpsDisconnectButton;
    QLabel *label_5;
    QComboBox *gpsPortCombo;
    QLabel *gpsStatusLabel;
    QCheckBox *gpsAutoCheckBox;
    QVBoxLayout *verticalLayout_5;
    QLabel *label_26;
    QPushButton *imuConnectButton;
    QPushButton *imuDisconnectButton;
    QLabel *label_2;
    QComboBox *imuPortCombo;
    QLabel *imuStatusLabel;
    QCheckBox *imuAutoCheckBox;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_27;
    QPushButton *cameraConnectButton;
    QPushButton *cameraDisconnectButton;
    QLabel *label_28;
    QComboBox *cameraPortCombo;
    QLabel *camerasStatusLabel;
    QCheckBox *cameraAutoCheckBox;
    QPushButton *allDevicesConnectButton;
    QPushButton *allDevicesDisconnectButton;
    QWidget *CalibrationTab;
    QLabel *calibCameraLabel;
    QLabel *calibLaserLabel;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *verticalLayout_4;
    QPushButton *calibGetDataButton;
    QPushButton *calibResetButton;
    QLabel *label_29;
    QLabel *calibIndexLabel;
    QMenuBar *mainMenu;

    void setupUi(QMainWindow *TrobotQtClass)
    {
        if (TrobotQtClass->objectName().isEmpty())
            TrobotQtClass->setObjectName(QString::fromUtf8("TrobotQtClass"));
        TrobotQtClass->resize(829, 705);
        TrobotQtClass->setLocale(QLocale(QLocale::English, QLocale::UnitedStates));
        centralWidget = new QWidget(TrobotQtClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        mainTab = new QTabWidget(centralWidget);
        mainTab->setObjectName(QString::fromUtf8("mainTab"));
        mainTab->setGeometry(QRect(0, 0, 831, 671));
        cameraTab = new QWidget();
        cameraTab->setObjectName(QString::fromUtf8("cameraTab"));
        cameraLabel = new QLabel(cameraTab);
        cameraLabel->setObjectName(QString::fromUtf8("cameraLabel"));
        cameraLabel->setGeometry(QRect(30, 60, 640, 480));
        cameraLabel->setFrameShape(QFrame::StyledPanel);
        cameraNewWindowButton = new QPushButton(cameraTab);
        cameraNewWindowButton->setObjectName(QString::fromUtf8("cameraNewWindowButton"));
        cameraNewWindowButton->setGeometry(QRect(680, 60, 121, 23));
        mainTab->addTab(cameraTab, QString());
        imuTab = new QWidget();
        imuTab->setObjectName(QString::fromUtf8("imuTab"));
        imuDisplayLabel = new QLabel(imuTab);
        imuDisplayLabel->setObjectName(QString::fromUtf8("imuDisplayLabel"));
        imuDisplayLabel->setGeometry(QRect(10, 40, 640, 480));
        imuDisplayLabel->setFrameShape(QFrame::StyledPanel);
        imuValuesGroupBox = new QGroupBox(imuTab);
        imuValuesGroupBox->setObjectName(QString::fromUtf8("imuValuesGroupBox"));
        imuValuesGroupBox->setGeometry(QRect(680, 150, 141, 451));
        accelGroupBox = new QGroupBox(imuValuesGroupBox);
        accelGroupBox->setObjectName(QString::fromUtf8("accelGroupBox"));
        accelGroupBox->setGeometry(QRect(0, 20, 141, 101));
        gridLayoutWidget_7 = new QWidget(accelGroupBox);
        gridLayoutWidget_7->setObjectName(QString::fromUtf8("gridLayoutWidget_7"));
        gridLayoutWidget_7->setGeometry(QRect(0, 20, 141, 85));
        gridLayout_5 = new QGridLayout(gridLayoutWidget_7);
        gridLayout_5->setSpacing(6);
        gridLayout_5->setContentsMargins(11, 11, 11, 11);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        gridLayout_5->setContentsMargins(0, 0, 0, 0);
        accelZcheckBox = new QCheckBox(gridLayoutWidget_7);
        accelZcheckBox->setObjectName(QString::fromUtf8("accelZcheckBox"));
        accelZcheckBox->setChecked(true);

        gridLayout_5->addWidget(accelZcheckBox, 4, 0, 1, 1);

        label_12 = new QLabel(gridLayoutWidget_7);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        gridLayout_5->addWidget(label_12, 1, 1, 1, 1);

        accelScaleLineEdit = new QLineEdit(gridLayoutWidget_7);
        accelScaleLineEdit->setObjectName(QString::fromUtf8("accelScaleLineEdit"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(40);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(accelScaleLineEdit->sizePolicy().hasHeightForWidth());
        accelScaleLineEdit->setSizePolicy(sizePolicy);
        accelScaleLineEdit->setMaximumSize(QSize(60, 16777215));

        gridLayout_5->addWidget(accelScaleLineEdit, 3, 1, 1, 1);

        accelYcheckBox = new QCheckBox(gridLayoutWidget_7);
        accelYcheckBox->setObjectName(QString::fromUtf8("accelYcheckBox"));
        accelYcheckBox->setChecked(true);

        gridLayout_5->addWidget(accelYcheckBox, 3, 0, 1, 1);

        accelXcheckBox = new QCheckBox(gridLayoutWidget_7);
        accelXcheckBox->setObjectName(QString::fromUtf8("accelXcheckBox"));
        accelXcheckBox->setChecked(true);

        gridLayout_5->addWidget(accelXcheckBox, 1, 0, 1, 1);

        gyroGroupBox = new QGroupBox(imuValuesGroupBox);
        gyroGroupBox->setObjectName(QString::fromUtf8("gyroGroupBox"));
        gyroGroupBox->setGeometry(QRect(0, 130, 141, 101));
        gridLayoutWidget_9 = new QWidget(gyroGroupBox);
        gridLayoutWidget_9->setObjectName(QString::fromUtf8("gridLayoutWidget_9"));
        gridLayoutWidget_9->setGeometry(QRect(0, 20, 141, 85));
        gridLayout_7 = new QGridLayout(gridLayoutWidget_9);
        gridLayout_7->setSpacing(6);
        gridLayout_7->setContentsMargins(11, 11, 11, 11);
        gridLayout_7->setObjectName(QString::fromUtf8("gridLayout_7"));
        gridLayout_7->setContentsMargins(0, 0, 0, 0);
        gyroXcheckBox = new QCheckBox(gridLayoutWidget_9);
        gyroXcheckBox->setObjectName(QString::fromUtf8("gyroXcheckBox"));
        gyroXcheckBox->setChecked(true);

        gridLayout_7->addWidget(gyroXcheckBox, 0, 0, 1, 1);

        gyroYcheckBox = new QCheckBox(gridLayoutWidget_9);
        gyroYcheckBox->setObjectName(QString::fromUtf8("gyroYcheckBox"));
        gyroYcheckBox->setChecked(true);

        gridLayout_7->addWidget(gyroYcheckBox, 1, 0, 1, 1);

        gyroZcheckBox = new QCheckBox(gridLayoutWidget_9);
        gyroZcheckBox->setObjectName(QString::fromUtf8("gyroZcheckBox"));
        gyroZcheckBox->setChecked(true);

        gridLayout_7->addWidget(gyroZcheckBox, 2, 0, 1, 1);

        label_15 = new QLabel(gridLayoutWidget_9);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        gridLayout_7->addWidget(label_15, 0, 1, 1, 1);

        gyroScaleLineEdit = new QLineEdit(gridLayoutWidget_9);
        gyroScaleLineEdit->setObjectName(QString::fromUtf8("gyroScaleLineEdit"));
        gyroScaleLineEdit->setMaximumSize(QSize(60, 16777215));

        gridLayout_7->addWidget(gyroScaleLineEdit, 1, 1, 1, 1);

        magGroupBox = new QGroupBox(imuValuesGroupBox);
        magGroupBox->setObjectName(QString::fromUtf8("magGroupBox"));
        magGroupBox->setGeometry(QRect(0, 240, 141, 101));
        gridLayoutWidget_8 = new QWidget(magGroupBox);
        gridLayoutWidget_8->setObjectName(QString::fromUtf8("gridLayoutWidget_8"));
        gridLayoutWidget_8->setGeometry(QRect(0, 20, 141, 85));
        gridLayout_6 = new QGridLayout(gridLayoutWidget_8);
        gridLayout_6->setSpacing(6);
        gridLayout_6->setContentsMargins(11, 11, 11, 11);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        gridLayout_6->setContentsMargins(0, 0, 0, 0);
        magXcheckBox = new QCheckBox(gridLayoutWidget_8);
        magXcheckBox->setObjectName(QString::fromUtf8("magXcheckBox"));
        magXcheckBox->setChecked(true);

        gridLayout_6->addWidget(magXcheckBox, 0, 0, 1, 1);

        magYcheckBox = new QCheckBox(gridLayoutWidget_8);
        magYcheckBox->setObjectName(QString::fromUtf8("magYcheckBox"));
        magYcheckBox->setChecked(true);

        gridLayout_6->addWidget(magYcheckBox, 1, 0, 1, 1);

        magZcheckBox = new QCheckBox(gridLayoutWidget_8);
        magZcheckBox->setObjectName(QString::fromUtf8("magZcheckBox"));
        magZcheckBox->setChecked(true);

        gridLayout_6->addWidget(magZcheckBox, 2, 0, 1, 1);

        label_13 = new QLabel(gridLayoutWidget_8);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        gridLayout_6->addWidget(label_13, 0, 1, 1, 1);

        magScaleLineEdit = new QLineEdit(gridLayoutWidget_8);
        magScaleLineEdit->setObjectName(QString::fromUtf8("magScaleLineEdit"));
        magScaleLineEdit->setMaximumSize(QSize(60, 16777215));

        gridLayout_6->addWidget(magScaleLineEdit, 1, 1, 1, 1);

        eulerGroupBox = new QGroupBox(imuValuesGroupBox);
        eulerGroupBox->setObjectName(QString::fromUtf8("eulerGroupBox"));
        eulerGroupBox->setGeometry(QRect(0, 350, 141, 101));
        gridLayoutWidget_10 = new QWidget(eulerGroupBox);
        gridLayoutWidget_10->setObjectName(QString::fromUtf8("gridLayoutWidget_10"));
        gridLayoutWidget_10->setGeometry(QRect(0, 20, 149, 85));
        gridLayout_8 = new QGridLayout(gridLayoutWidget_10);
        gridLayout_8->setSpacing(6);
        gridLayout_8->setContentsMargins(11, 11, 11, 11);
        gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
        gridLayout_8->setContentsMargins(0, 0, 0, 0);
        eulerRollCheckBox = new QCheckBox(gridLayoutWidget_10);
        eulerRollCheckBox->setObjectName(QString::fromUtf8("eulerRollCheckBox"));
        eulerRollCheckBox->setChecked(true);

        gridLayout_8->addWidget(eulerRollCheckBox, 0, 0, 1, 1);

        eulerPitchCheckBox = new QCheckBox(gridLayoutWidget_10);
        eulerPitchCheckBox->setObjectName(QString::fromUtf8("eulerPitchCheckBox"));
        eulerPitchCheckBox->setChecked(true);

        gridLayout_8->addWidget(eulerPitchCheckBox, 1, 0, 1, 1);

        eulerYawCheckBox = new QCheckBox(gridLayoutWidget_10);
        eulerYawCheckBox->setObjectName(QString::fromUtf8("eulerYawCheckBox"));
        eulerYawCheckBox->setChecked(true);

        gridLayout_8->addWidget(eulerYawCheckBox, 2, 0, 1, 1);

        label_16 = new QLabel(gridLayoutWidget_10);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        gridLayout_8->addWidget(label_16, 0, 1, 1, 1);

        eulerScaleLineEdit = new QLineEdit(gridLayoutWidget_10);
        eulerScaleLineEdit->setObjectName(QString::fromUtf8("eulerScaleLineEdit"));
        eulerScaleLineEdit->setMaximumSize(QSize(60, 16777215));

        gridLayout_8->addWidget(eulerScaleLineEdit, 1, 1, 1, 1);

        gridLayoutWidget = new QWidget(imuTab);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(20, 540, 301, 83));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        imuClearButton = new QPushButton(gridLayoutWidget);
        imuClearButton->setObjectName(QString::fromUtf8("imuClearButton"));
        imuClearButton->setEnabled(false);

        gridLayout->addWidget(imuClearButton, 1, 1, 1, 1);

        imuStartButton = new QPushButton(gridLayoutWidget);
        imuStartButton->setObjectName(QString::fromUtf8("imuStartButton"));
        imuStartButton->setEnabled(false);

        gridLayout->addWidget(imuStartButton, 1, 0, 1, 1);

        imuStopButton = new QPushButton(gridLayoutWidget);
        imuStopButton->setObjectName(QString::fromUtf8("imuStopButton"));
        imuStopButton->setEnabled(false);

        gridLayout->addWidget(imuStopButton, 2, 0, 1, 1);

        gridLayoutWidget_6 = new QWidget(imuTab);
        gridLayoutWidget_6->setObjectName(QString::fromUtf8("gridLayoutWidget_6"));
        gridLayoutWidget_6->setGeometry(QRect(360, 540, 241, 90));
        gridLayout_4 = new QGridLayout(gridLayoutWidget_6);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        label_10 = new QLabel(gridLayoutWidget_6);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label_10->sizePolicy().hasHeightForWidth());
        label_10->setSizePolicy(sizePolicy1);

        gridLayout_4->addWidget(label_10, 0, 0, 1, 1);

        imuTimeScaleLineEdit = new QLineEdit(gridLayoutWidget_6);
        imuTimeScaleLineEdit->setObjectName(QString::fromUtf8("imuTimeScaleLineEdit"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(60);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(imuTimeScaleLineEdit->sizePolicy().hasHeightForWidth());
        imuTimeScaleLineEdit->setSizePolicy(sizePolicy2);
        imuTimeScaleLineEdit->setMaximumSize(QSize(60, 16777215));

        gridLayout_4->addWidget(imuTimeScaleLineEdit, 2, 0, 1, 1);

        label_11 = new QLabel(gridLayoutWidget_6);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_4->addWidget(label_11, 0, 1, 1, 1);

        imuAutoScrollCheckBox = new QCheckBox(gridLayoutWidget_6);
        imuAutoScrollCheckBox->setObjectName(QString::fromUtf8("imuAutoScrollCheckBox"));

        gridLayout_4->addWidget(imuAutoScrollCheckBox, 2, 1, 1, 1);

        imuTimeScrollBar = new QScrollBar(imuTab);
        imuTimeScrollBar->setObjectName(QString::fromUtf8("imuTimeScrollBar"));
        imuTimeScrollBar->setEnabled(false);
        imuTimeScrollBar->setGeometry(QRect(10, 520, 641, 16));
        imuTimeScrollBar->setMinimum(-10);
        imuTimeScrollBar->setMaximum(2000);
        imuTimeScrollBar->setOrientation(Qt::Horizontal);
        imuValueScrollBar = new QScrollBar(imuTab);
        imuValueScrollBar->setObjectName(QString::fromUtf8("imuValueScrollBar"));
        imuValueScrollBar->setEnabled(false);
        imuValueScrollBar->setGeometry(QRect(650, 39, 20, 481));
        imuValueScrollBar->setMinimum(-1000);
        imuValueScrollBar->setMaximum(1000);
        imuValueScrollBar->setOrientation(Qt::Vertical);
        mainTab->addTab(imuTab, QString());
        gpsTab = new QWidget();
        gpsTab->setObjectName(QString::fromUtf8("gpsTab"));
        gpsDisplayLabel = new QLabel(gpsTab);
        gpsDisplayLabel->setObjectName(QString::fromUtf8("gpsDisplayLabel"));
        gpsDisplayLabel->setGeometry(QRect(20, 50, 640, 480));
        gpsDisplayLabel->setFrameShape(QFrame::StyledPanel);
        gpsValuesGroupBox = new QGroupBox(gpsTab);
        gpsValuesGroupBox->setObjectName(QString::fromUtf8("gpsValuesGroupBox"));
        gpsValuesGroupBox->setGeometry(QRect(670, 379, 131, 111));
        verticalLayoutWidget_8 = new QWidget(gpsValuesGroupBox);
        verticalLayoutWidget_8->setObjectName(QString::fromUtf8("verticalLayoutWidget_8"));
        verticalLayoutWidget_8->setGeometry(QRect(10, 20, 111, 81));
        verticalLayout_8 = new QVBoxLayout(verticalLayoutWidget_8);
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setContentsMargins(11, 11, 11, 11);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        verticalLayout_8->setContentsMargins(0, 0, 0, 0);
        longtitudeCheckBox = new QCheckBox(verticalLayoutWidget_8);
        longtitudeCheckBox->setObjectName(QString::fromUtf8("longtitudeCheckBox"));

        verticalLayout_8->addWidget(longtitudeCheckBox);

        lattitudeCheckBox = new QCheckBox(verticalLayoutWidget_8);
        lattitudeCheckBox->setObjectName(QString::fromUtf8("lattitudeCheckBox"));

        verticalLayout_8->addWidget(lattitudeCheckBox);

        verticalLayoutWidget_9 = new QWidget(gpsTab);
        verticalLayoutWidget_9->setObjectName(QString::fromUtf8("verticalLayoutWidget_9"));
        verticalLayoutWidget_9->setGeometry(QRect(670, 180, 131, 181));
        verticalLayout_9 = new QVBoxLayout(verticalLayoutWidget_9);
        verticalLayout_9->setSpacing(6);
        verticalLayout_9->setContentsMargins(11, 11, 11, 11);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        verticalLayout_9->setContentsMargins(0, 0, 0, 0);
        label_7 = new QLabel(verticalLayoutWidget_9);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        verticalLayout_9->addWidget(label_7);

        gpsSatelitesLabel = new QLabel(verticalLayoutWidget_9);
        gpsSatelitesLabel->setObjectName(QString::fromUtf8("gpsSatelitesLabel"));

        verticalLayout_9->addWidget(gpsSatelitesLabel);

        label_9 = new QLabel(verticalLayoutWidget_9);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        verticalLayout_9->addWidget(label_9);

        gpsFixLabel = new QLabel(verticalLayoutWidget_9);
        gpsFixLabel->setObjectName(QString::fromUtf8("gpsFixLabel"));

        verticalLayout_9->addWidget(gpsFixLabel);

        label_18 = new QLabel(verticalLayoutWidget_9);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        verticalLayout_9->addWidget(label_18);

        gpsXLabel = new QLabel(verticalLayoutWidget_9);
        gpsXLabel->setObjectName(QString::fromUtf8("gpsXLabel"));

        verticalLayout_9->addWidget(gpsXLabel);

        gpsYLabel = new QLabel(verticalLayoutWidget_9);
        gpsYLabel->setObjectName(QString::fromUtf8("gpsYLabel"));

        verticalLayout_9->addWidget(gpsYLabel);

        gpsZeroPointButton = new QPushButton(gpsTab);
        gpsZeroPointButton->setObjectName(QString::fromUtf8("gpsZeroPointButton"));
        gpsZeroPointButton->setGeometry(QRect(680, 500, 121, 27));
        mainTab->addTab(gpsTab, QString());
        sensorsTab = new QWidget();
        sensorsTab->setObjectName(QString::fromUtf8("sensorsTab"));
        sensorsAnalogGroupBox = new QGroupBox(sensorsTab);
        sensorsAnalogGroupBox->setObjectName(QString::fromUtf8("sensorsAnalogGroupBox"));
        sensorsAnalogGroupBox->setGeometry(QRect(30, 30, 241, 181));
        gridLayoutWidget_4 = new QWidget(sensorsAnalogGroupBox);
        gridLayoutWidget_4->setObjectName(QString::fromUtf8("gridLayoutWidget_4"));
        gridLayoutWidget_4->setGeometry(QRect(10, 20, 221, 151));
        sensorsAnalogLayout = new QGridLayout(gridLayoutWidget_4);
        sensorsAnalogLayout->setSpacing(6);
        sensorsAnalogLayout->setContentsMargins(11, 11, 11, 11);
        sensorsAnalogLayout->setObjectName(QString::fromUtf8("sensorsAnalogLayout"));
        sensorsAnalogLayout->setContentsMargins(0, 0, 0, 0);
        label_19 = new QLabel(gridLayoutWidget_4);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        sensorsAnalogLayout->addWidget(label_19, 0, 0, 1, 1);

        label_20 = new QLabel(gridLayoutWidget_4);
        label_20->setObjectName(QString::fromUtf8("label_20"));

        sensorsAnalogLayout->addWidget(label_20, 0, 1, 1, 1);

        label_21 = new QLabel(gridLayoutWidget_4);
        label_21->setObjectName(QString::fromUtf8("label_21"));

        sensorsAnalogLayout->addWidget(label_21, 0, 2, 1, 1);

        sensorsDigitalGroupBox = new QGroupBox(sensorsTab);
        sensorsDigitalGroupBox->setObjectName(QString::fromUtf8("sensorsDigitalGroupBox"));
        sensorsDigitalGroupBox->setGeometry(QRect(20, 240, 691, 111));
        gridLayoutWidget_5 = new QWidget(sensorsDigitalGroupBox);
        gridLayoutWidget_5->setObjectName(QString::fromUtf8("gridLayoutWidget_5"));
        gridLayoutWidget_5->setGeometry(QRect(10, 20, 671, 80));
        sensorsDigitalLayout = new QGridLayout(gridLayoutWidget_5);
        sensorsDigitalLayout->setSpacing(6);
        sensorsDigitalLayout->setContentsMargins(11, 11, 11, 11);
        sensorsDigitalLayout->setObjectName(QString::fromUtf8("sensorsDigitalLayout"));
        sensorsDigitalLayout->setContentsMargins(0, 0, 0, 0);
        label_14 = new QLabel(gridLayoutWidget_5);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        sensorsDigitalLayout->addWidget(label_14, 0, 1, 1, 1);

        label_31 = new QLabel(gridLayoutWidget_5);
        label_31->setObjectName(QString::fromUtf8("label_31"));

        sensorsDigitalLayout->addWidget(label_31, 1, 1, 1, 1);

        label_32 = new QLabel(gridLayoutWidget_5);
        label_32->setObjectName(QString::fromUtf8("label_32"));

        sensorsDigitalLayout->addWidget(label_32, 2, 1, 1, 1);

        mainTab->addTab(sensorsTab, QString());
        robotDriveTab = new QWidget();
        robotDriveTab->setObjectName(QString::fromUtf8("robotDriveTab"));
        gridLayoutWidget_2 = new QWidget(robotDriveTab);
        gridLayoutWidget_2->setObjectName(QString::fromUtf8("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(60, 340, 321, 201));
        gridLayout_2 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        robotDriveLeftButton = new QPushButton(gridLayoutWidget_2);
        robotDriveLeftButton->setObjectName(QString::fromUtf8("robotDriveLeftButton"));
        robotDriveLeftButton->setEnabled(false);
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(robotDriveLeftButton->sizePolicy().hasHeightForWidth());
        robotDriveLeftButton->setSizePolicy(sizePolicy3);
        robotDriveLeftButton->setMinimumSize(QSize(75, 75));
        robotDriveLeftButton->setMaximumSize(QSize(75, 75));

        gridLayout_2->addWidget(robotDriveLeftButton, 1, 0, 1, 1);

        robotDriveDownButton = new QPushButton(gridLayoutWidget_2);
        robotDriveDownButton->setObjectName(QString::fromUtf8("robotDriveDownButton"));
        robotDriveDownButton->setEnabled(false);
        sizePolicy3.setHeightForWidth(robotDriveDownButton->sizePolicy().hasHeightForWidth());
        robotDriveDownButton->setSizePolicy(sizePolicy3);
        robotDriveDownButton->setMinimumSize(QSize(75, 75));
        robotDriveDownButton->setMaximumSize(QSize(75, 75));

        gridLayout_2->addWidget(robotDriveDownButton, 1, 1, 1, 1);

        robotDriveRightButton = new QPushButton(gridLayoutWidget_2);
        robotDriveRightButton->setObjectName(QString::fromUtf8("robotDriveRightButton"));
        robotDriveRightButton->setEnabled(false);
        sizePolicy3.setHeightForWidth(robotDriveRightButton->sizePolicy().hasHeightForWidth());
        robotDriveRightButton->setSizePolicy(sizePolicy3);
        robotDriveRightButton->setMinimumSize(QSize(75, 75));
        robotDriveRightButton->setMaximumSize(QSize(75, 75));

        gridLayout_2->addWidget(robotDriveRightButton, 1, 2, 1, 1);

        robotDriveUpButton = new QPushButton(gridLayoutWidget_2);
        robotDriveUpButton->setObjectName(QString::fromUtf8("robotDriveUpButton"));
        robotDriveUpButton->setEnabled(false);
        sizePolicy3.setHeightForWidth(robotDriveUpButton->sizePolicy().hasHeightForWidth());
        robotDriveUpButton->setSizePolicy(sizePolicy3);
        robotDriveUpButton->setMinimumSize(QSize(75, 75));
        robotDriveUpButton->setMaximumSize(QSize(75, 75));

        gridLayout_2->addWidget(robotDriveUpButton, 0, 1, 1, 1);

        robotDriveMotorCtrlGroupBox = new QGroupBox(robotDriveTab);
        robotDriveMotorCtrlGroupBox->setObjectName(QString::fromUtf8("robotDriveMotorCtrlGroupBox"));
        robotDriveMotorCtrlGroupBox->setGeometry(QRect(50, 30, 201, 291));
        gridLayoutWidget_3 = new QWidget(robotDriveMotorCtrlGroupBox);
        gridLayoutWidget_3->setObjectName(QString::fromUtf8("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(9, 19, 183, 261));
        gridLayout_3 = new QGridLayout(gridLayoutWidget_3);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        robotDriveRightMotorSlider = new QSlider(gridLayoutWidget_3);
        robotDriveRightMotorSlider->setObjectName(QString::fromUtf8("robotDriveRightMotorSlider"));
        robotDriveRightMotorSlider->setEnabled(false);
        robotDriveRightMotorSlider->setMinimum(-1000);
        robotDriveRightMotorSlider->setMaximum(1000);
        robotDriveRightMotorSlider->setOrientation(Qt::Vertical);

        gridLayout_3->addWidget(robotDriveRightMotorSlider, 1, 2, 1, 1);

        label = new QLabel(gridLayoutWidget_3);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_3->addWidget(label, 0, 0, 1, 1);

        label_3 = new QLabel(gridLayoutWidget_3);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_3->addWidget(label_3, 0, 2, 1, 1);

        robotDriveLeftMotorLabel = new QLabel(gridLayoutWidget_3);
        robotDriveLeftMotorLabel->setObjectName(QString::fromUtf8("robotDriveLeftMotorLabel"));
        robotDriveLeftMotorLabel->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(robotDriveLeftMotorLabel, 2, 0, 1, 1);

        robotDriveRightMotorLabel = new QLabel(gridLayoutWidget_3);
        robotDriveRightMotorLabel->setObjectName(QString::fromUtf8("robotDriveRightMotorLabel"));
        robotDriveRightMotorLabel->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(robotDriveRightMotorLabel, 2, 2, 1, 1);

        robotDriveLeftMotorStopButton = new QPushButton(gridLayoutWidget_3);
        robotDriveLeftMotorStopButton->setObjectName(QString::fromUtf8("robotDriveLeftMotorStopButton"));
        robotDriveLeftMotorStopButton->setEnabled(false);

        gridLayout_3->addWidget(robotDriveLeftMotorStopButton, 3, 0, 1, 1);

        robotDriveRightMotorStopButton = new QPushButton(gridLayoutWidget_3);
        robotDriveRightMotorStopButton->setObjectName(QString::fromUtf8("robotDriveRightMotorStopButton"));
        robotDriveRightMotorStopButton->setEnabled(false);

        gridLayout_3->addWidget(robotDriveRightMotorStopButton, 3, 2, 1, 1);

        robotDriveLeftMotorSlider = new QSlider(gridLayoutWidget_3);
        robotDriveLeftMotorSlider->setObjectName(QString::fromUtf8("robotDriveLeftMotorSlider"));
        robotDriveLeftMotorSlider->setEnabled(false);
        robotDriveLeftMotorSlider->setLayoutDirection(Qt::LeftToRight);
        robotDriveLeftMotorSlider->setAutoFillBackground(false);
        robotDriveLeftMotorSlider->setMinimum(-1000);
        robotDriveLeftMotorSlider->setMaximum(1000);
        robotDriveLeftMotorSlider->setOrientation(Qt::Vertical);

        gridLayout_3->addWidget(robotDriveLeftMotorSlider, 1, 0, 1, 1);

        groupBox = new QGroupBox(robotDriveTab);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(280, 30, 251, 281));
        robotDriveSteeringScrollBar = new QScrollBar(groupBox);
        robotDriveSteeringScrollBar->setObjectName(QString::fromUtf8("robotDriveSteeringScrollBar"));
        robotDriveSteeringScrollBar->setEnabled(false);
        robotDriveSteeringScrollBar->setGeometry(QRect(70, 140, 160, 16));
        robotDriveSteeringScrollBar->setMinimum(-100);
        robotDriveSteeringScrollBar->setMaximum(100);
        robotDriveSteeringScrollBar->setOrientation(Qt::Horizontal);
        robotDriveSteeringScrollBar->setInvertedAppearance(false);
        robotDriveThrottleScrollBar = new QScrollBar(groupBox);
        robotDriveThrottleScrollBar->setObjectName(QString::fromUtf8("robotDriveThrottleScrollBar"));
        robotDriveThrottleScrollBar->setEnabled(false);
        robotDriveThrottleScrollBar->setGeometry(QRect(20, 30, 20, 211));
        robotDriveThrottleScrollBar->setMinimum(-1000);
        robotDriveThrottleScrollBar->setMaximum(1000);
        robotDriveThrottleScrollBar->setOrientation(Qt::Vertical);
        robotDriveThrottleScrollBar->setInvertedAppearance(true);
        robotDriveThrottleScrollBar->setInvertedControls(true);
        robotDriveSteeringLabel = new QLabel(groupBox);
        robotDriveSteeringLabel->setObjectName(QString::fromUtf8("robotDriveSteeringLabel"));
        robotDriveSteeringLabel->setGeometry(QRect(120, 170, 65, 17));
        robotDriveSteeringLabel->setAlignment(Qt::AlignCenter);
        robotDriveThrottleLabel = new QLabel(groupBox);
        robotDriveThrottleLabel->setObjectName(QString::fromUtf8("robotDriveThrottleLabel"));
        robotDriveThrottleLabel->setGeometry(QRect(10, 250, 41, 20));
        robotDriveThrottleLabel->setAlignment(Qt::AlignCenter);
        robotDriveThrottleStopButton = new QPushButton(robotDriveTab);
        robotDriveThrottleStopButton->setObjectName(QString::fromUtf8("robotDriveThrottleStopButton"));
        robotDriveThrottleStopButton->setGeometry(QRect(260, 300, 97, 27));
        mainTab->addTab(robotDriveTab, QString());
        hokuyoTab = new QWidget();
        hokuyoTab->setObjectName(QString::fromUtf8("hokuyoTab"));
        hokuyoDisplayLabel = new QLabel(hokuyoTab);
        hokuyoDisplayLabel->setObjectName(QString::fromUtf8("hokuyoDisplayLabel"));
        hokuyoDisplayLabel->setGeometry(QRect(10, 20, 600, 600));
        hokuyoDisplayLabel->setFrameShape(QFrame::StyledPanel);
        mainTab->addTab(hokuyoTab, QString());
        recordTab = new QWidget();
        recordTab->setObjectName(QString::fromUtf8("recordTab"));
        saRateGroupBox = new QGroupBox(recordTab);
        saRateGroupBox->setObjectName(QString::fromUtf8("saRateGroupBox"));
        saRateGroupBox->setGeometry(QRect(30, 30, 201, 391));
        verticalLayoutWidget = new QWidget(saRateGroupBox);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 30, 181, 347));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        includeHokuyoCheckBox = new QCheckBox(verticalLayoutWidget);
        includeHokuyoCheckBox->setObjectName(QString::fromUtf8("includeHokuyoCheckBox"));
        includeHokuyoCheckBox->setCheckable(true);
        includeHokuyoCheckBox->setChecked(false);

        verticalLayout->addWidget(includeHokuyoCheckBox);

        saRateHokuyoLineEdit = new QLineEdit(verticalLayoutWidget);
        saRateHokuyoLineEdit->setObjectName(QString::fromUtf8("saRateHokuyoLineEdit"));

        verticalLayout->addWidget(saRateHokuyoLineEdit);

        includeEncodersCheckBox = new QCheckBox(verticalLayoutWidget);
        includeEncodersCheckBox->setObjectName(QString::fromUtf8("includeEncodersCheckBox"));
        includeEncodersCheckBox->setChecked(true);

        verticalLayout->addWidget(includeEncodersCheckBox);

        saRateEncodersLineEdit = new QLineEdit(verticalLayoutWidget);
        saRateEncodersLineEdit->setObjectName(QString::fromUtf8("saRateEncodersLineEdit"));

        verticalLayout->addWidget(saRateEncodersLineEdit);

        includeGpsCheckBox = new QCheckBox(verticalLayoutWidget);
        includeGpsCheckBox->setObjectName(QString::fromUtf8("includeGpsCheckBox"));
        includeGpsCheckBox->setChecked(true);

        verticalLayout->addWidget(includeGpsCheckBox);

        saRateGpsLineEdit = new QLineEdit(verticalLayoutWidget);
        saRateGpsLineEdit->setObjectName(QString::fromUtf8("saRateGpsLineEdit"));

        verticalLayout->addWidget(saRateGpsLineEdit);

        includeCamerasCheckBox = new QCheckBox(verticalLayoutWidget);
        includeCamerasCheckBox->setObjectName(QString::fromUtf8("includeCamerasCheckBox"));
        includeCamerasCheckBox->setChecked(false);

        verticalLayout->addWidget(includeCamerasCheckBox);

        saRateCamerasLineEdit = new QLineEdit(verticalLayoutWidget);
        saRateCamerasLineEdit->setObjectName(QString::fromUtf8("saRateCamerasLineEdit"));

        verticalLayout->addWidget(saRateCamerasLineEdit);

        includeImuCheckBox = new QCheckBox(verticalLayoutWidget);
        includeImuCheckBox->setObjectName(QString::fromUtf8("includeImuCheckBox"));
        includeImuCheckBox->setChecked(true);

        verticalLayout->addWidget(includeImuCheckBox);

        saRateImuLineEdit = new QLineEdit(verticalLayoutWidget);
        saRateImuLineEdit->setObjectName(QString::fromUtf8("saRateImuLineEdit"));

        verticalLayout->addWidget(saRateImuLineEdit);

        groupBox_2 = new QGroupBox(recordTab);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(240, 29, 191, 261));
        verticalLayoutWidget_2 = new QWidget(groupBox_2);
        verticalLayoutWidget_2->setObjectName(QString::fromUtf8("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(10, 29, 171, 211));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_17 = new QLabel(verticalLayoutWidget_2);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        sizePolicy1.setHeightForWidth(label_17->sizePolicy().hasHeightForWidth());
        label_17->setSizePolicy(sizePolicy1);

        verticalLayout_2->addWidget(label_17);

        recPathLineEdit = new QLineEdit(verticalLayoutWidget_2);
        recPathLineEdit->setObjectName(QString::fromUtf8("recPathLineEdit"));

        verticalLayout_2->addWidget(recPathLineEdit);

        startRecButton = new QPushButton(verticalLayoutWidget_2);
        startRecButton->setObjectName(QString::fromUtf8("startRecButton"));

        verticalLayout_2->addWidget(startRecButton);

        pauseResumeRecButton = new QPushButton(verticalLayoutWidget_2);
        pauseResumeRecButton->setObjectName(QString::fromUtf8("pauseResumeRecButton"));

        verticalLayout_2->addWidget(pauseResumeRecButton);

        stopRecButon = new QPushButton(verticalLayoutWidget_2);
        stopRecButon->setObjectName(QString::fromUtf8("stopRecButon"));

        verticalLayout_2->addWidget(stopRecButon);

        recStatusLabel = new QLabel(verticalLayoutWidget_2);
        recStatusLabel->setObjectName(QString::fromUtf8("recStatusLabel"));
        sizePolicy1.setHeightForWidth(recStatusLabel->sizePolicy().hasHeightForWidth());
        recStatusLabel->setSizePolicy(sizePolicy1);

        verticalLayout_2->addWidget(recStatusLabel);

        mainTab->addTab(recordTab, QString());
        DevicesTab = new QWidget();
        DevicesTab->setObjectName(QString::fromUtf8("DevicesTab"));
        horizontalLayoutWidget = new QWidget(DevicesTab);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(30, 29, 770, 250));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout_11 = new QVBoxLayout();
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
        label_22 = new QLabel(horizontalLayoutWidget);
        label_22->setObjectName(QString::fromUtf8("label_22"));

        verticalLayout_11->addWidget(label_22);

        hokuyoConnectButton = new QPushButton(horizontalLayoutWidget);
        hokuyoConnectButton->setObjectName(QString::fromUtf8("hokuyoConnectButton"));

        verticalLayout_11->addWidget(hokuyoConnectButton);

        hokuyoDisconnectButton = new QPushButton(horizontalLayoutWidget);
        hokuyoDisconnectButton->setObjectName(QString::fromUtf8("hokuyoDisconnectButton"));

        verticalLayout_11->addWidget(hokuyoDisconnectButton);

        label_8 = new QLabel(horizontalLayoutWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        verticalLayout_11->addWidget(label_8);

        hokuyoPortCombo = new QComboBox(horizontalLayoutWidget);
        hokuyoPortCombo->setObjectName(QString::fromUtf8("hokuyoPortCombo"));

        verticalLayout_11->addWidget(hokuyoPortCombo);

        hokuyoStatusLabel = new QLabel(horizontalLayoutWidget);
        hokuyoStatusLabel->setObjectName(QString::fromUtf8("hokuyoStatusLabel"));

        verticalLayout_11->addWidget(hokuyoStatusLabel);

        hokuyoAutoCheckBox = new QCheckBox(horizontalLayoutWidget);
        hokuyoAutoCheckBox->setObjectName(QString::fromUtf8("hokuyoAutoCheckBox"));

        verticalLayout_11->addWidget(hokuyoAutoCheckBox);


        horizontalLayout->addLayout(verticalLayout_11);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        label_23 = new QLabel(horizontalLayoutWidget);
        label_23->setObjectName(QString::fromUtf8("label_23"));

        verticalLayout_6->addWidget(label_23);

        robotDriveConnectButton = new QPushButton(horizontalLayoutWidget);
        robotDriveConnectButton->setObjectName(QString::fromUtf8("robotDriveConnectButton"));

        verticalLayout_6->addWidget(robotDriveConnectButton);

        robotDriveDisconnectButton = new QPushButton(horizontalLayoutWidget);
        robotDriveDisconnectButton->setObjectName(QString::fromUtf8("robotDriveDisconnectButton"));

        verticalLayout_6->addWidget(robotDriveDisconnectButton);

        label_4 = new QLabel(horizontalLayoutWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout_6->addWidget(label_4);

        robotDriversLeftPortCombo = new QComboBox(horizontalLayoutWidget);
        robotDriversLeftPortCombo->setObjectName(QString::fromUtf8("robotDriversLeftPortCombo"));

        verticalLayout_6->addWidget(robotDriversLeftPortCombo);

        label_35 = new QLabel(horizontalLayoutWidget);
        label_35->setObjectName(QString::fromUtf8("label_35"));

        verticalLayout_6->addWidget(label_35);

        robotDriversRightPortCombo = new QComboBox(horizontalLayoutWidget);
        robotDriversRightPortCombo->setObjectName(QString::fromUtf8("robotDriversRightPortCombo"));

        verticalLayout_6->addWidget(robotDriversRightPortCombo);

        robotDriveStatusLabel = new QLabel(horizontalLayoutWidget);
        robotDriveStatusLabel->setObjectName(QString::fromUtf8("robotDriveStatusLabel"));

        verticalLayout_6->addWidget(robotDriveStatusLabel);

        robotDriveAutoCheckBox = new QCheckBox(horizontalLayoutWidget);
        robotDriveAutoCheckBox->setObjectName(QString::fromUtf8("robotDriveAutoCheckBox"));

        verticalLayout_6->addWidget(robotDriveAutoCheckBox);


        horizontalLayout->addLayout(verticalLayout_6);

        verticalLayout_12 = new QVBoxLayout();
        verticalLayout_12->setSpacing(6);
        verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
        label_30 = new QLabel(horizontalLayoutWidget);
        label_30->setObjectName(QString::fromUtf8("label_30"));

        verticalLayout_12->addWidget(label_30);

        encodersConnectButton = new QPushButton(horizontalLayoutWidget);
        encodersConnectButton->setObjectName(QString::fromUtf8("encodersConnectButton"));

        verticalLayout_12->addWidget(encodersConnectButton);

        encodersDisconnectButton = new QPushButton(horizontalLayoutWidget);
        encodersDisconnectButton->setObjectName(QString::fromUtf8("encodersDisconnectButton"));

        verticalLayout_12->addWidget(encodersDisconnectButton);

        label_33 = new QLabel(horizontalLayoutWidget);
        label_33->setObjectName(QString::fromUtf8("label_33"));

        verticalLayout_12->addWidget(label_33);

        encodersPortCombo = new QComboBox(horizontalLayoutWidget);
        encodersPortCombo->setObjectName(QString::fromUtf8("encodersPortCombo"));

        verticalLayout_12->addWidget(encodersPortCombo);

        label_34 = new QLabel(horizontalLayoutWidget);
        label_34->setObjectName(QString::fromUtf8("label_34"));

        verticalLayout_12->addWidget(label_34);

        encodersAutoCheckBox = new QCheckBox(horizontalLayoutWidget);
        encodersAutoCheckBox->setObjectName(QString::fromUtf8("encodersAutoCheckBox"));

        verticalLayout_12->addWidget(encodersAutoCheckBox);


        horizontalLayout->addLayout(verticalLayout_12);

        verticalLayout_10 = new QVBoxLayout();
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        label_24 = new QLabel(horizontalLayoutWidget);
        label_24->setObjectName(QString::fromUtf8("label_24"));

        verticalLayout_10->addWidget(label_24);

        sensorsConnectButton = new QPushButton(horizontalLayoutWidget);
        sensorsConnectButton->setObjectName(QString::fromUtf8("sensorsConnectButton"));

        verticalLayout_10->addWidget(sensorsConnectButton);

        sensorsDisconnectButton = new QPushButton(horizontalLayoutWidget);
        sensorsDisconnectButton->setObjectName(QString::fromUtf8("sensorsDisconnectButton"));

        verticalLayout_10->addWidget(sensorsDisconnectButton);

        label_6 = new QLabel(horizontalLayoutWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        verticalLayout_10->addWidget(label_6);

        sensorsPortCombo = new QComboBox(horizontalLayoutWidget);
        sensorsPortCombo->setObjectName(QString::fromUtf8("sensorsPortCombo"));

        verticalLayout_10->addWidget(sensorsPortCombo);

        sensorsStatusLabel = new QLabel(horizontalLayoutWidget);
        sensorsStatusLabel->setObjectName(QString::fromUtf8("sensorsStatusLabel"));

        verticalLayout_10->addWidget(sensorsStatusLabel);

        sensorsAutoCheckBox = new QCheckBox(horizontalLayoutWidget);
        sensorsAutoCheckBox->setObjectName(QString::fromUtf8("sensorsAutoCheckBox"));

        verticalLayout_10->addWidget(sensorsAutoCheckBox);


        horizontalLayout->addLayout(verticalLayout_10);

        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        label_25 = new QLabel(horizontalLayoutWidget);
        label_25->setObjectName(QString::fromUtf8("label_25"));

        verticalLayout_7->addWidget(label_25);

        gpsConnectButton = new QPushButton(horizontalLayoutWidget);
        gpsConnectButton->setObjectName(QString::fromUtf8("gpsConnectButton"));

        verticalLayout_7->addWidget(gpsConnectButton);

        gpsDisconnectButton = new QPushButton(horizontalLayoutWidget);
        gpsDisconnectButton->setObjectName(QString::fromUtf8("gpsDisconnectButton"));

        verticalLayout_7->addWidget(gpsDisconnectButton);

        label_5 = new QLabel(horizontalLayoutWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        verticalLayout_7->addWidget(label_5);

        gpsPortCombo = new QComboBox(horizontalLayoutWidget);
        gpsPortCombo->setObjectName(QString::fromUtf8("gpsPortCombo"));

        verticalLayout_7->addWidget(gpsPortCombo);

        gpsStatusLabel = new QLabel(horizontalLayoutWidget);
        gpsStatusLabel->setObjectName(QString::fromUtf8("gpsStatusLabel"));

        verticalLayout_7->addWidget(gpsStatusLabel);

        gpsAutoCheckBox = new QCheckBox(horizontalLayoutWidget);
        gpsAutoCheckBox->setObjectName(QString::fromUtf8("gpsAutoCheckBox"));

        verticalLayout_7->addWidget(gpsAutoCheckBox);


        horizontalLayout->addLayout(verticalLayout_7);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        label_26 = new QLabel(horizontalLayoutWidget);
        label_26->setObjectName(QString::fromUtf8("label_26"));

        verticalLayout_5->addWidget(label_26);

        imuConnectButton = new QPushButton(horizontalLayoutWidget);
        imuConnectButton->setObjectName(QString::fromUtf8("imuConnectButton"));

        verticalLayout_5->addWidget(imuConnectButton);

        imuDisconnectButton = new QPushButton(horizontalLayoutWidget);
        imuDisconnectButton->setObjectName(QString::fromUtf8("imuDisconnectButton"));

        verticalLayout_5->addWidget(imuDisconnectButton);

        label_2 = new QLabel(horizontalLayoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_5->addWidget(label_2);

        imuPortCombo = new QComboBox(horizontalLayoutWidget);
        imuPortCombo->setObjectName(QString::fromUtf8("imuPortCombo"));

        verticalLayout_5->addWidget(imuPortCombo);

        imuStatusLabel = new QLabel(horizontalLayoutWidget);
        imuStatusLabel->setObjectName(QString::fromUtf8("imuStatusLabel"));

        verticalLayout_5->addWidget(imuStatusLabel);

        imuAutoCheckBox = new QCheckBox(horizontalLayoutWidget);
        imuAutoCheckBox->setObjectName(QString::fromUtf8("imuAutoCheckBox"));

        verticalLayout_5->addWidget(imuAutoCheckBox);


        horizontalLayout->addLayout(verticalLayout_5);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        label_27 = new QLabel(horizontalLayoutWidget);
        label_27->setObjectName(QString::fromUtf8("label_27"));

        verticalLayout_3->addWidget(label_27);

        cameraConnectButton = new QPushButton(horizontalLayoutWidget);
        cameraConnectButton->setObjectName(QString::fromUtf8("cameraConnectButton"));

        verticalLayout_3->addWidget(cameraConnectButton);

        cameraDisconnectButton = new QPushButton(horizontalLayoutWidget);
        cameraDisconnectButton->setObjectName(QString::fromUtf8("cameraDisconnectButton"));

        verticalLayout_3->addWidget(cameraDisconnectButton);

        label_28 = new QLabel(horizontalLayoutWidget);
        label_28->setObjectName(QString::fromUtf8("label_28"));

        verticalLayout_3->addWidget(label_28);

        cameraPortCombo = new QComboBox(horizontalLayoutWidget);
        cameraPortCombo->setObjectName(QString::fromUtf8("cameraPortCombo"));

        verticalLayout_3->addWidget(cameraPortCombo);

        camerasStatusLabel = new QLabel(horizontalLayoutWidget);
        camerasStatusLabel->setObjectName(QString::fromUtf8("camerasStatusLabel"));

        verticalLayout_3->addWidget(camerasStatusLabel);

        cameraAutoCheckBox = new QCheckBox(horizontalLayoutWidget);
        cameraAutoCheckBox->setObjectName(QString::fromUtf8("cameraAutoCheckBox"));

        verticalLayout_3->addWidget(cameraAutoCheckBox);


        horizontalLayout->addLayout(verticalLayout_3);

        allDevicesConnectButton = new QPushButton(DevicesTab);
        allDevicesConnectButton->setObjectName(QString::fromUtf8("allDevicesConnectButton"));
        allDevicesConnectButton->setGeometry(QRect(40, 310, 98, 27));
        allDevicesDisconnectButton = new QPushButton(DevicesTab);
        allDevicesDisconnectButton->setObjectName(QString::fromUtf8("allDevicesDisconnectButton"));
        allDevicesDisconnectButton->setGeometry(QRect(40, 340, 98, 27));
        mainTab->addTab(DevicesTab, QString());
        CalibrationTab = new QWidget();
        CalibrationTab->setObjectName(QString::fromUtf8("CalibrationTab"));
        calibCameraLabel = new QLabel(CalibrationTab);
        calibCameraLabel->setObjectName(QString::fromUtf8("calibCameraLabel"));
        calibCameraLabel->setGeometry(QRect(20, 30, 320, 240));
        calibCameraLabel->setFrameShape(QFrame::Box);
        calibLaserLabel = new QLabel(CalibrationTab);
        calibLaserLabel->setObjectName(QString::fromUtf8("calibLaserLabel"));
        calibLaserLabel->setGeometry(QRect(390, 30, 300, 300));
        calibLaserLabel->setFrameShape(QFrame::Box);
        verticalLayoutWidget_3 = new QWidget(CalibrationTab);
        verticalLayoutWidget_3->setObjectName(QString::fromUtf8("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(20, 330, 160, 121));
        verticalLayout_4 = new QVBoxLayout(verticalLayoutWidget_3);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        calibGetDataButton = new QPushButton(verticalLayoutWidget_3);
        calibGetDataButton->setObjectName(QString::fromUtf8("calibGetDataButton"));

        verticalLayout_4->addWidget(calibGetDataButton);

        calibResetButton = new QPushButton(verticalLayoutWidget_3);
        calibResetButton->setObjectName(QString::fromUtf8("calibResetButton"));

        verticalLayout_4->addWidget(calibResetButton);

        label_29 = new QLabel(CalibrationTab);
        label_29->setObjectName(QString::fromUtf8("label_29"));
        label_29->setGeometry(QRect(230, 340, 101, 17));
        calibIndexLabel = new QLabel(CalibrationTab);
        calibIndexLabel->setObjectName(QString::fromUtf8("calibIndexLabel"));
        calibIndexLabel->setGeometry(QRect(230, 360, 66, 17));
        mainTab->addTab(CalibrationTab, QString());
        TrobotQtClass->setCentralWidget(centralWidget);
        mainMenu = new QMenuBar(TrobotQtClass);
        mainMenu->setObjectName(QString::fromUtf8("mainMenu"));
        mainMenu->setGeometry(QRect(0, 0, 829, 25));
        TrobotQtClass->setMenuBar(mainMenu);

        retranslateUi(TrobotQtClass);

        mainTab->setCurrentIndex(7);


        QMetaObject::connectSlotsByName(TrobotQtClass);
    } // setupUi

    void retranslateUi(QMainWindow *TrobotQtClass)
    {
        TrobotQtClass->setWindowTitle(QApplication::translate("TrobotQtClass", "Trobot", 0, QApplication::UnicodeUTF8));
        cameraLabel->setText(QString());
        cameraNewWindowButton->setText(QApplication::translate("TrobotQtClass", "Open in new window", 0, QApplication::UnicodeUTF8));
        mainTab->setTabText(mainTab->indexOf(cameraTab), QApplication::translate("TrobotQtClass", "Camera", 0, QApplication::UnicodeUTF8));
        imuDisplayLabel->setText(QString());
        imuValuesGroupBox->setTitle(QApplication::translate("TrobotQtClass", "Values to display", 0, QApplication::UnicodeUTF8));
        accelGroupBox->setTitle(QApplication::translate("TrobotQtClass", "Accelerometer", 0, QApplication::UnicodeUTF8));
        accelZcheckBox->setText(QApplication::translate("TrobotQtClass", "Z", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("TrobotQtClass", "Value scale", 0, QApplication::UnicodeUTF8));
        accelScaleLineEdit->setText(QApplication::translate("TrobotQtClass", "10", 0, QApplication::UnicodeUTF8));
        accelYcheckBox->setText(QApplication::translate("TrobotQtClass", "Y", 0, QApplication::UnicodeUTF8));
        accelXcheckBox->setText(QApplication::translate("TrobotQtClass", "X", 0, QApplication::UnicodeUTF8));
        gyroGroupBox->setTitle(QApplication::translate("TrobotQtClass", "Gyroscope", 0, QApplication::UnicodeUTF8));
        gyroXcheckBox->setText(QApplication::translate("TrobotQtClass", "X", 0, QApplication::UnicodeUTF8));
        gyroYcheckBox->setText(QApplication::translate("TrobotQtClass", "Y", 0, QApplication::UnicodeUTF8));
        gyroZcheckBox->setText(QApplication::translate("TrobotQtClass", "Z", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("TrobotQtClass", "Value scale", 0, QApplication::UnicodeUTF8));
        gyroScaleLineEdit->setText(QApplication::translate("TrobotQtClass", "10", 0, QApplication::UnicodeUTF8));
        magGroupBox->setTitle(QApplication::translate("TrobotQtClass", "Magnetometer", 0, QApplication::UnicodeUTF8));
        magXcheckBox->setText(QApplication::translate("TrobotQtClass", "X", 0, QApplication::UnicodeUTF8));
        magYcheckBox->setText(QApplication::translate("TrobotQtClass", "Y", 0, QApplication::UnicodeUTF8));
        magZcheckBox->setText(QApplication::translate("TrobotQtClass", "Z", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("TrobotQtClass", "Value scale", 0, QApplication::UnicodeUTF8));
        magScaleLineEdit->setText(QApplication::translate("TrobotQtClass", "10", 0, QApplication::UnicodeUTF8));
        eulerGroupBox->setTitle(QApplication::translate("TrobotQtClass", "Euler angles", 0, QApplication::UnicodeUTF8));
        eulerRollCheckBox->setText(QApplication::translate("TrobotQtClass", "Roll", 0, QApplication::UnicodeUTF8));
        eulerPitchCheckBox->setText(QApplication::translate("TrobotQtClass", "Pitch", 0, QApplication::UnicodeUTF8));
        eulerYawCheckBox->setText(QApplication::translate("TrobotQtClass", "Yaw", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("TrobotQtClass", "Value scale", 0, QApplication::UnicodeUTF8));
        eulerScaleLineEdit->setText(QApplication::translate("TrobotQtClass", "10", 0, QApplication::UnicodeUTF8));
        imuClearButton->setText(QApplication::translate("TrobotQtClass", "Clear", 0, QApplication::UnicodeUTF8));
        imuStartButton->setText(QApplication::translate("TrobotQtClass", "Start", 0, QApplication::UnicodeUTF8));
        imuStopButton->setText(QApplication::translate("TrobotQtClass", "Stop", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("TrobotQtClass", "Time scale", 0, QApplication::UnicodeUTF8));
        imuTimeScaleLineEdit->setText(QApplication::translate("TrobotQtClass", "20", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("TrobotQtClass", "AutoScroll", 0, QApplication::UnicodeUTF8));
        imuAutoScrollCheckBox->setText(QApplication::translate("TrobotQtClass", "Auto scroll", 0, QApplication::UnicodeUTF8));
        mainTab->setTabText(mainTab->indexOf(imuTab), QApplication::translate("TrobotQtClass", "IMU", 0, QApplication::UnicodeUTF8));
        gpsDisplayLabel->setText(QString());
        gpsValuesGroupBox->setTitle(QApplication::translate("TrobotQtClass", "Values to display", 0, QApplication::UnicodeUTF8));
        longtitudeCheckBox->setText(QApplication::translate("TrobotQtClass", "Longtitude", 0, QApplication::UnicodeUTF8));
        lattitudeCheckBox->setText(QApplication::translate("TrobotQtClass", "Lattitude", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("TrobotQtClass", "Satelites in use:", 0, QApplication::UnicodeUTF8));
        gpsSatelitesLabel->setText(QApplication::translate("TrobotQtClass", "satelites", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("TrobotQtClass", "Fix status:", 0, QApplication::UnicodeUTF8));
        gpsFixLabel->setText(QApplication::translate("TrobotQtClass", "fix?", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("TrobotQtClass", "Position:", 0, QApplication::UnicodeUTF8));
        gpsXLabel->setText(QApplication::translate("TrobotQtClass", "X", 0, QApplication::UnicodeUTF8));
        gpsYLabel->setText(QApplication::translate("TrobotQtClass", "Y", 0, QApplication::UnicodeUTF8));
        gpsZeroPointButton->setText(QApplication::translate("TrobotQtClass", "Set origin here", 0, QApplication::UnicodeUTF8));
        mainTab->setTabText(mainTab->indexOf(gpsTab), QApplication::translate("TrobotQtClass", "GPS", 0, QApplication::UnicodeUTF8));
        sensorsAnalogGroupBox->setTitle(QApplication::translate("TrobotQtClass", "Analog inputs", 0, QApplication::UnicodeUTF8));
        label_19->setText(QApplication::translate("TrobotQtClass", "Input", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("TrobotQtClass", "Value", 0, QApplication::UnicodeUTF8));
        label_21->setText(QApplication::translate("TrobotQtClass", "Sensor type", 0, QApplication::UnicodeUTF8));
        sensorsDigitalGroupBox->setTitle(QApplication::translate("TrobotQtClass", "GroupBox", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("TrobotQtClass", "Pin no", 0, QApplication::UnicodeUTF8));
        label_31->setText(QApplication::translate("TrobotQtClass", "Output", 0, QApplication::UnicodeUTF8));
        label_32->setText(QApplication::translate("TrobotQtClass", "State", 0, QApplication::UnicodeUTF8));
        mainTab->setTabText(mainTab->indexOf(sensorsTab), QApplication::translate("TrobotQtClass", "Sensors", 0, QApplication::UnicodeUTF8));
        robotDriveLeftButton->setText(QApplication::translate("TrobotQtClass", "Left", 0, QApplication::UnicodeUTF8));
        robotDriveDownButton->setText(QApplication::translate("TrobotQtClass", "Down", 0, QApplication::UnicodeUTF8));
        robotDriveRightButton->setText(QApplication::translate("TrobotQtClass", "Right", 0, QApplication::UnicodeUTF8));
        robotDriveUpButton->setText(QApplication::translate("TrobotQtClass", "Up", 0, QApplication::UnicodeUTF8));
        robotDriveMotorCtrlGroupBox->setTitle(QApplication::translate("TrobotQtClass", "Motors control", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("TrobotQtClass", "Left motor", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("TrobotQtClass", "Right motor", 0, QApplication::UnicodeUTF8));
        robotDriveLeftMotorLabel->setText(QApplication::translate("TrobotQtClass", "0", 0, QApplication::UnicodeUTF8));
        robotDriveRightMotorLabel->setText(QApplication::translate("TrobotQtClass", "0", 0, QApplication::UnicodeUTF8));
        robotDriveLeftMotorStopButton->setText(QApplication::translate("TrobotQtClass", "Stop", 0, QApplication::UnicodeUTF8));
        robotDriveRightMotorStopButton->setText(QApplication::translate("TrobotQtClass", "Stop", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("TrobotQtClass", "Steering", 0, QApplication::UnicodeUTF8));
        robotDriveSteeringLabel->setText(QApplication::translate("TrobotQtClass", "0", 0, QApplication::UnicodeUTF8));
        robotDriveThrottleLabel->setText(QApplication::translate("TrobotQtClass", "0", 0, QApplication::UnicodeUTF8));
        robotDriveThrottleStopButton->setText(QApplication::translate("TrobotQtClass", "Stop", 0, QApplication::UnicodeUTF8));
        mainTab->setTabText(mainTab->indexOf(robotDriveTab), QApplication::translate("TrobotQtClass", "Robot's drive", 0, QApplication::UnicodeUTF8));
        hokuyoDisplayLabel->setText(QString());
        mainTab->setTabText(mainTab->indexOf(hokuyoTab), QApplication::translate("TrobotQtClass", "Hokuyo", 0, QApplication::UnicodeUTF8));
        saRateGroupBox->setTitle(QApplication::translate("TrobotQtClass", "Sampling rate [Sa/s]", 0, QApplication::UnicodeUTF8));
        includeHokuyoCheckBox->setText(QApplication::translate("TrobotQtClass", "Hokuyo", 0, QApplication::UnicodeUTF8));
        saRateHokuyoLineEdit->setText(QApplication::translate("TrobotQtClass", "10", 0, QApplication::UnicodeUTF8));
        includeEncodersCheckBox->setText(QApplication::translate("TrobotQtClass", "Encoders", 0, QApplication::UnicodeUTF8));
        saRateEncodersLineEdit->setText(QApplication::translate("TrobotQtClass", "100", 0, QApplication::UnicodeUTF8));
        includeGpsCheckBox->setText(QApplication::translate("TrobotQtClass", "GPS", 0, QApplication::UnicodeUTF8));
        saRateGpsLineEdit->setText(QApplication::translate("TrobotQtClass", "1", 0, QApplication::UnicodeUTF8));
        includeCamerasCheckBox->setText(QApplication::translate("TrobotQtClass", "Cameras", 0, QApplication::UnicodeUTF8));
        saRateCamerasLineEdit->setText(QApplication::translate("TrobotQtClass", "1", 0, QApplication::UnicodeUTF8));
        includeImuCheckBox->setText(QApplication::translate("TrobotQtClass", "IMU", 0, QApplication::UnicodeUTF8));
        saRateImuLineEdit->setText(QApplication::translate("TrobotQtClass", "100", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("TrobotQtClass", "Control", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("TrobotQtClass", "File path", 0, QApplication::UnicodeUTF8));
        recPathLineEdit->setText(QApplication::translate("TrobotQtClass", "data", 0, QApplication::UnicodeUTF8));
        startRecButton->setText(QApplication::translate("TrobotQtClass", "Start", 0, QApplication::UnicodeUTF8));
        pauseResumeRecButton->setText(QApplication::translate("TrobotQtClass", "Pause", 0, QApplication::UnicodeUTF8));
        stopRecButon->setText(QApplication::translate("TrobotQtClass", "Stop", 0, QApplication::UnicodeUTF8));
        recStatusLabel->setText(QApplication::translate("TrobotQtClass", "status", 0, QApplication::UnicodeUTF8));
        mainTab->setTabText(mainTab->indexOf(recordTab), QApplication::translate("TrobotQtClass", "Recording", 0, QApplication::UnicodeUTF8));
        label_22->setText(QApplication::translate("TrobotQtClass", "Hokuyo", 0, QApplication::UnicodeUTF8));
        hokuyoConnectButton->setText(QApplication::translate("TrobotQtClass", "Connect", 0, QApplication::UnicodeUTF8));
        hokuyoDisconnectButton->setText(QApplication::translate("TrobotQtClass", "Disconnect", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("TrobotQtClass", "Device:", 0, QApplication::UnicodeUTF8));
        hokuyoStatusLabel->setText(QApplication::translate("TrobotQtClass", "Status", 0, QApplication::UnicodeUTF8));
        hokuyoAutoCheckBox->setText(QApplication::translate("TrobotQtClass", "Auto", 0, QApplication::UnicodeUTF8));
        label_23->setText(QApplication::translate("TrobotQtClass", "Robot's Drive", 0, QApplication::UnicodeUTF8));
        robotDriveConnectButton->setText(QApplication::translate("TrobotQtClass", "Connect", 0, QApplication::UnicodeUTF8));
        robotDriveDisconnectButton->setText(QApplication::translate("TrobotQtClass", "Disconnect", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("TrobotQtClass", "Left device:", 0, QApplication::UnicodeUTF8));
        label_35->setText(QApplication::translate("TrobotQtClass", "Right device:", 0, QApplication::UnicodeUTF8));
        robotDriveStatusLabel->setText(QApplication::translate("TrobotQtClass", "Status", 0, QApplication::UnicodeUTF8));
        robotDriveAutoCheckBox->setText(QApplication::translate("TrobotQtClass", "Auto", 0, QApplication::UnicodeUTF8));
        label_30->setText(QApplication::translate("TrobotQtClass", "Encoders", 0, QApplication::UnicodeUTF8));
        encodersConnectButton->setText(QApplication::translate("TrobotQtClass", "Connect", 0, QApplication::UnicodeUTF8));
        encodersDisconnectButton->setText(QApplication::translate("TrobotQtClass", "Disconnect", 0, QApplication::UnicodeUTF8));
        label_33->setText(QApplication::translate("TrobotQtClass", "Device:", 0, QApplication::UnicodeUTF8));
        label_34->setText(QApplication::translate("TrobotQtClass", "Status", 0, QApplication::UnicodeUTF8));
        encodersAutoCheckBox->setText(QApplication::translate("TrobotQtClass", "Auto", 0, QApplication::UnicodeUTF8));
        label_24->setText(QApplication::translate("TrobotQtClass", "Sensors", 0, QApplication::UnicodeUTF8));
        sensorsConnectButton->setText(QApplication::translate("TrobotQtClass", "Connect", 0, QApplication::UnicodeUTF8));
        sensorsDisconnectButton->setText(QApplication::translate("TrobotQtClass", "Disconnect", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("TrobotQtClass", "Device:", 0, QApplication::UnicodeUTF8));
        sensorsStatusLabel->setText(QApplication::translate("TrobotQtClass", "Status", 0, QApplication::UnicodeUTF8));
        sensorsAutoCheckBox->setText(QApplication::translate("TrobotQtClass", "Auto", 0, QApplication::UnicodeUTF8));
        label_25->setText(QApplication::translate("TrobotQtClass", "GPS", 0, QApplication::UnicodeUTF8));
        gpsConnectButton->setText(QApplication::translate("TrobotQtClass", "Connect", 0, QApplication::UnicodeUTF8));
        gpsDisconnectButton->setText(QApplication::translate("TrobotQtClass", "Disconnect", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("TrobotQtClass", "Device:", 0, QApplication::UnicodeUTF8));
        gpsStatusLabel->setText(QApplication::translate("TrobotQtClass", "Status", 0, QApplication::UnicodeUTF8));
        gpsAutoCheckBox->setText(QApplication::translate("TrobotQtClass", "Auto", 0, QApplication::UnicodeUTF8));
        label_26->setText(QApplication::translate("TrobotQtClass", "IMU", 0, QApplication::UnicodeUTF8));
        imuConnectButton->setText(QApplication::translate("TrobotQtClass", "Connect", 0, QApplication::UnicodeUTF8));
        imuDisconnectButton->setText(QApplication::translate("TrobotQtClass", "Disconnect", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("TrobotQtClass", "Device:", 0, QApplication::UnicodeUTF8));
        imuStatusLabel->setText(QApplication::translate("TrobotQtClass", "Status", 0, QApplication::UnicodeUTF8));
        imuAutoCheckBox->setText(QApplication::translate("TrobotQtClass", "Auto", 0, QApplication::UnicodeUTF8));
        label_27->setText(QApplication::translate("TrobotQtClass", "Camera", 0, QApplication::UnicodeUTF8));
        cameraConnectButton->setText(QApplication::translate("TrobotQtClass", "Connect", 0, QApplication::UnicodeUTF8));
        cameraDisconnectButton->setText(QApplication::translate("TrobotQtClass", "Disconnect", 0, QApplication::UnicodeUTF8));
        label_28->setText(QApplication::translate("TrobotQtClass", "Device:", 0, QApplication::UnicodeUTF8));
        camerasStatusLabel->setText(QApplication::translate("TrobotQtClass", "Status", 0, QApplication::UnicodeUTF8));
        cameraAutoCheckBox->setText(QApplication::translate("TrobotQtClass", "Auto", 0, QApplication::UnicodeUTF8));
        allDevicesConnectButton->setText(QApplication::translate("TrobotQtClass", "Connect", 0, QApplication::UnicodeUTF8));
        allDevicesDisconnectButton->setText(QApplication::translate("TrobotQtClass", "Disconnect", 0, QApplication::UnicodeUTF8));
        mainTab->setTabText(mainTab->indexOf(DevicesTab), QApplication::translate("TrobotQtClass", "Devices", 0, QApplication::UnicodeUTF8));
        calibCameraLabel->setText(QString());
        calibLaserLabel->setText(QString());
        calibGetDataButton->setText(QApplication::translate("TrobotQtClass", "Get Data", 0, QApplication::UnicodeUTF8));
        calibResetButton->setText(QApplication::translate("TrobotQtClass", "Reset", 0, QApplication::UnicodeUTF8));
        label_29->setText(QApplication::translate("TrobotQtClass", "Current index", 0, QApplication::UnicodeUTF8));
        calibIndexLabel->setText(QApplication::translate("TrobotQtClass", "0", 0, QApplication::UnicodeUTF8));
        mainTab->setTabText(mainTab->indexOf(CalibrationTab), QApplication::translate("TrobotQtClass", "Calibration", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class TrobotQtClass: public Ui_TrobotQtClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TROBOTQT_H

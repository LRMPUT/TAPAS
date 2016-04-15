/****************************************************************************
** Meta object code from reading C++ file 'QtRobotDrive.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../TapasQt/QtRobotDrive.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QtRobotDrive.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_QtRobotDrive[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      14,   13,   13,   13, 0x0a,
      26,   13,   13,   13, 0x0a,
      39,   13,   13,   13, 0x0a,
      48,   13,   13,   13, 0x0a,
      58,   13,   13,   13, 0x0a,
      74,   13,   13,   13, 0x0a,
      91,   13,   13,   13, 0x0a,
     102,   98,   13,   13, 0x0a,
     123,   98,   13,   13, 0x0a,
     144,   98,   13,   13, 0x0a,
     165,   13,   13,   13, 0x0a,
     182,   13,   13,   13, 0x0a,
     200,   13,   13,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_QtRobotDrive[] = {
    "QtRobotDrive\0\0goForward()\0goBackward()\0"
    "goLeft()\0goRight()\0leftMotorStop()\0"
    "rightMotorStop()\0stop()\0val\0"
    "motorValChanged(int)\0throttleChanged(int)\0"
    "steeringChanged(int)\0openRobotDrive()\0"
    "closeRobotDrive()\0updateState()\0"
};

void QtRobotDrive::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QtRobotDrive *_t = static_cast<QtRobotDrive *>(_o);
        switch (_id) {
        case 0: _t->goForward(); break;
        case 1: _t->goBackward(); break;
        case 2: _t->goLeft(); break;
        case 3: _t->goRight(); break;
        case 4: _t->leftMotorStop(); break;
        case 5: _t->rightMotorStop(); break;
        case 6: _t->stop(); break;
        case 7: _t->motorValChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->throttleChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->steeringChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->openRobotDrive(); break;
        case 11: _t->closeRobotDrive(); break;
        case 12: _t->updateState(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData QtRobotDrive::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject QtRobotDrive::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_QtRobotDrive,
      qt_meta_data_QtRobotDrive, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &QtRobotDrive::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *QtRobotDrive::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *QtRobotDrive::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_QtRobotDrive))
        return static_cast<void*>(const_cast< QtRobotDrive*>(this));
    return QObject::qt_metacast(_clname);
}

int QtRobotDrive::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

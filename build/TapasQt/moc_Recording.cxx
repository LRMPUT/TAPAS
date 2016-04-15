/****************************************************************************
** Meta object code from reading C++ file 'Recording.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../TapasQt/Recording.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Recording.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Recording[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      11,   10,   10,   10, 0x08,
      27,   10,   10,   10, 0x08,
      45,   10,   10,   10, 0x08,
      58,   10,   10,   10, 0x08,
      71,   10,   10,   10, 0x08,
      87,   10,   10,   10, 0x08,
     109,   10,   10,   10, 0x08,
     128,   10,   10,   10, 0x0a,
     139,   10,   10,   10, 0x0a,
     156,   10,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Recording[] = {
    "Recording\0\0getDataHokuyo()\0getDataEncoders()\0"
    "getDataGps()\0getDataImu()\0getDataCamera()\0"
    "getDataEstimatedPos()\0getGoalDirGlobal()\0"
    "startRec()\0pauseResumeRec()\0stopRec()\0"
};

void Recording::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Recording *_t = static_cast<Recording *>(_o);
        switch (_id) {
        case 0: _t->getDataHokuyo(); break;
        case 1: _t->getDataEncoders(); break;
        case 2: _t->getDataGps(); break;
        case 3: _t->getDataImu(); break;
        case 4: _t->getDataCamera(); break;
        case 5: _t->getDataEstimatedPos(); break;
        case 6: _t->getGoalDirGlobal(); break;
        case 7: _t->startRec(); break;
        case 8: _t->pauseResumeRec(); break;
        case 9: _t->stopRec(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData Recording::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Recording::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Recording,
      qt_meta_data_Recording, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Recording::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Recording::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Recording::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Recording))
        return static_cast<void*>(const_cast< Recording*>(this));
    return QObject::qt_metacast(_clname);
}

int Recording::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

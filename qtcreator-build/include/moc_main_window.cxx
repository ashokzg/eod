/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created: Wed Apr 3 20:09:59 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../teamb_ui/include/main_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_teamb_ui__MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      22,   21,   21,   21, 0x05,
      38,   21,   21,   21, 0x05,
      54,   50,   21,   21, 0x05,
      85,   21,   21,   21, 0x05,

 // slots: signature, parameters, type, tag, flags
     100,   21,   21,   21, 0x0a,
     115,   21,   21,   21, 0x0a,
     137,   21,   21,   21, 0x0a,
     152,   21,   21,   21, 0x0a,
     170,   21,   21,   21, 0x0a,
     190,   21,   21,   21, 0x0a,
     207,   21,   21,   21, 0x0a,
     231,  227,   21,   21, 0x0a,
     253,  227,   21,   21, 0x0a,
     275,   21,   21,   21, 0x0a,
     289,   50,   21,   21, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_teamb_ui__MainWindow[] = {
    "teamb_ui::MainWindow\0\0changemode(int)\0"
    "updateRos()\0,,,\0mouseOverInfo(int,int,int,int)\0"
    "sendState(int)\0connectToROS()\0"
    "toggleManualMode(int)\0updateWindow()\0"
    "confirmTracking()\0resetToIdleManual()\0"
    "setTargetInPic()\0updateLoggingView()\0"
    "img\0updateNewImg(cv::Mat)\0"
    "updateTrkImg(cv::Mat)\0updateLabel()\0"
    "paintRectangle(int,int,int,int)\0"
};

void teamb_ui::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->changemode((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->updateRos(); break;
        case 2: _t->mouseOverInfo((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 3: _t->sendState((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->connectToROS(); break;
        case 5: _t->toggleManualMode((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->updateWindow(); break;
        case 7: _t->confirmTracking(); break;
        case 8: _t->resetToIdleManual(); break;
        case 9: _t->setTargetInPic(); break;
        case 10: _t->updateLoggingView(); break;
        case 11: _t->updateNewImg((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 12: _t->updateTrkImg((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 13: _t->updateLabel(); break;
        case 14: _t->paintRectangle((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData teamb_ui::MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject teamb_ui::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_teamb_ui__MainWindow,
      qt_meta_data_teamb_ui__MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &teamb_ui::MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *teamb_ui::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *teamb_ui::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_teamb_ui__MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int teamb_ui::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void teamb_ui::MainWindow::changemode(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void teamb_ui::MainWindow::updateRos()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void teamb_ui::MainWindow::mouseOverInfo(int _t1, int _t2, int _t3, int _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void teamb_ui::MainWindow::sendState(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE

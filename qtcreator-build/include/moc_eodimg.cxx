/****************************************************************************
** Meta object code from reading C++ file 'eodimg.hpp'
**
** Created: Wed Apr 3 20:38:53 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../teamb_ui/include/eodimg.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'eodimg.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_eodImg[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,    8,    7,    7, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_eodImg[] = {
    "eodImg\0\0img\0updateImage(cv::Mat)\0"
};

void eodImg::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        eodImg *_t = static_cast<eodImg *>(_o);
        switch (_id) {
        case 0: _t->updateImage((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData eodImg::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject eodImg::staticMetaObject = {
    { &QGraphicsView::staticMetaObject, qt_meta_stringdata_eodImg,
      qt_meta_data_eodImg, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &eodImg::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *eodImg::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *eodImg::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_eodImg))
        return static_cast<void*>(const_cast< eodImg*>(this));
    return QGraphicsView::qt_metacast(_clname);
}

int eodImg::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGraphicsView::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

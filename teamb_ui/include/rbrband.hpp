#ifndef RBRBAND_H
#define RBRBAND_H

#include <QRubberBand>
#include <QStylePainter>
#include <QStyleOptionFocusRect>

class rbrBand : public QRubberBand
{
    Q_OBJECT
public:
    explicit rbrBand(QRubberBand::Shape, QWidget *parent = 0);
    QPalette palette;

public Q_SLOTS:

Q_SIGNALS:

protected:
    virtual void paintEvent(QPaintEvent *) ;
};


#endif // RBRBAND_H

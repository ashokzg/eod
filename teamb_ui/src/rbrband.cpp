#include "rbrband.hpp"

rbrBand::rbrBand(Shape s, QWidget *parent) :
    QRubberBand(s, parent){
    palette.setBrush(QPalette::Foreground, QBrush(Qt::red));
}

void rbrBand::paintEvent( QPaintEvent* ev)
{
        setPalette(palette);
        QStylePainter painter(this);
        QStyleOptionFocusRect option;
        option.initFrom( this );
        painter.drawControl(QStyle::CE_FocusFrame, option);
}

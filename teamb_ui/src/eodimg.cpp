#include "eodimg.hpp"
#include "main_window.hpp"
#include "rbrband.hpp"

eodImg::eodImg(QWidget *parent) :
    QGraphicsView(parent),rubber(new rbrBand(QRubberBand::Rectangle, this))
{
    x = 0;
    y = 0;
    xw = 0;
    yw = 0;
    rubber->setMouseTracking(true);
    currentlySelecting = false;
    scene = new QGraphicsScene(this);
    scene->addItem(&qAshImg);
    this->setScene(scene);

}

void eodImg::updateImage(cv::Mat img)
{    
    qAshImg.setPixmap(QPixmap::fromImage(eodImg::Mat2QImage(img)));
}

QImage eodImg::Mat2QImage(const cv::Mat3b &src)
{
        QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
        for (int y = 0; y < src.rows; ++y) {
                const cv::Vec3b *srcrow = src[y];
                QRgb *destrow = (QRgb*)dest.scanLine(y);
                for (int x = 0; x < src.cols; ++x) {
                        destrow[x] = qRgba(srcrow[x][2], srcrow[x][1], srcrow[x][0], 255);
                }
        }
        return dest;
}

void eodImg::mousePressEvent(QMouseEvent *ev){
    dragStart = ev->pos();
    QPointF pt = mapToScene(ev->pos());
    currentlySelecting = true;
    rubber->setGeometry(QRect(dragStart, QSize()));
    x= pt.x();
    y = pt.y();
    rubber->show();
}

void eodImg::mouseMoveEvent(QMouseEvent *ev)
{
    if (ev->buttons() & Qt::LeftButton)
        rubber->setGeometry(QRect(dragStart,ev->pos()).normalized());
        rubber->palette.setBrush(QPalette::Foreground, QBrush(Qt::yellow));

}

void eodImg::mouseReleaseEvent(QMouseEvent *ev)
{
    if(currentlySelecting == true)
    {
        //changed for sake of picture to green - Actual blue
    rubber->palette.setBrush(QPalette::Foreground, QBrush(Qt::green));
    rubber->show();
    xw = ev->pos().x()-x;
    yw = ev->pos().y()-y;

    }
}


#include "trkimg.hpp"
#include "main_window.hpp"

trkImg::trkImg(QWidget *parent) :
    QGraphicsView(parent)
{
    scene = new QGraphicsScene(this);
    scene->addItem(&qAshImg);
    this->setScene(scene);
    //scene.addItem(rect);
}

void trkImg::updateImage(cv::Mat img)
{    
    qAshImg.setPixmap(QPixmap::fromImage(trkImg::Mat2QImage(img)));
}

QImage trkImg::Mat2QImage(const cv::Mat3b &src)
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


void trkImg::paintEvent(QPaintEvent *ev){
    ev->ActivateControl;
    QPainter painter(viewport());
    QPen myPen(Qt::green);
    myPen.setWidth(4);
    painter.setPen(myPen);
    qDebug()<<x<<y<<xw<<yw;
    painter.drawRect(x,y,xw,yw);
}


void trkImg::input(int xI, int yI, int xwI, int ywI){
    x = xI;
    y = yI;
    xw = xwI;
    yw = ywI;
    //rect = new QGraphicsRectItem(x, y, xw, yw);
    //rect->setBrush(QBrush(Qt::green));

}


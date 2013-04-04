#include "trkimg.hpp"

trkImg::trkImg(QWidget *parent) :
    QGraphicsView(parent),rubber(new rbrBand(QRubberBand::Rectangle, this))
{
    scene = new QGraphicsScene(this);
    scene->addItem(&qAshImg);
    this->setScene(scene);
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



void trkImg::input(bool val,int x, int y, int xw, int yw){

    if(val){
        rubber->setGeometry(x,y,xw,yw);
        rubber->palette.setBrush(QPalette::Foreground, QBrush(Qt::green));
        rubber->show();
    }
    else
    {
        rubber->hide();
    }

}


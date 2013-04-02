#include "eodimg.hpp"
#include "main_window.hpp"

eodImg::eodImg(QWidget *parent) :
    QGraphicsView(parent),rubber(new QRubberBand(QRubberBand::Rectangle, this))
{
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

void eodImg::mousePressEvent(QMouseEvent *ev)
    {
    dragStart = ev->pos();
    QPointF pt = mapToScene(ev->pos());
    currentlySelecting = true;
    rubber->setGeometry(QRect(dragStart, QSize()));
    x= ev->pos().x();
    y = ev->pos().y();
    qDebug() <<pt.x()<<pt.y();
    rubber->show();

    }

void eodImg::mouseMoveEvent(QMouseEvent *ev)
    {
    if (ev->buttons() & Qt::LeftButton)
        rubber->setGeometry(QRect(dragStart,ev->pos()).normalized());

    QPalette palette;
    palette.setBrush(QPalette::Foreground, QBrush(Qt::red));
    rubber->setPalette(palette);

    }

void eodImg::mouseReleaseEvent(QMouseEvent *ev)
    {
    if(currentlySelecting == true)
    {


    QPalette palette;
    palette.setBrush(QPalette::Window, QBrush(Qt::green));


    rubber->setPalette(palette);

    rubber->show();
    xw = ev->pos().x()-x;
    yw = ev->pos().y()-y;
    qDebug() <<xw<<yw;

    QMessageBox msgBox;
    msgBox.setText("Are you sure of the selection?");
    msgBox.setInformativeText("Do you want to go ahead or reselect?");
    msgBox.setStandardButtons(QMessageBox::Apply | QMessageBox::Cancel);
    msgBox.setEscapeButton(QMessageBox::Close);
    msgBox.setDefaultButton(QMessageBox::Apply);
    ret = msgBox.exec();
    qDebug() <<ret;
    if(ret== 0x02000000)
    {
        QMessageBox msgBox2;
        msgBox2.setText("Please press Set Target to confirm ROI");
        msgBox2.setStandardButtons(QMessageBox::Ok);
        msgBox2.exec();
    }
    else
//    if(ret!= 0x02000000)
    {
        QMessageBox msgBox2;
        msgBox2.setText("Please re-select target!");
        msgBox2.exec();
        rubber->hide();
    }


    }
    }


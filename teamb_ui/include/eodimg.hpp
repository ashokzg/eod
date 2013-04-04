#ifndef EODIMG_H
#define EODIMG_H

#include <QLabel>
#include <QPointF>
#include <QMouseEvent>
#include <QMainWindow>
#include <QtCore>
#include <QtGui>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QMouseEvent>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rbrband.hpp"

class eodImg : public QGraphicsView
{
    Q_OBJECT
public:
    explicit eodImg(QWidget *parent = 0);
    int x,y,xw,yw;
    rbrBand* rubber;
    QRect getImgSelection(){
    return rubber->geometry();}
    void mousePressEvent(QMouseEvent * e);
    void mouseMoveEvent(QMouseEvent * e);
    void mouseReleaseEvent(QMouseEvent * e);

public Q_SLOTS:
    void updateImage(cv::Mat img);

Q_SIGNALS:

private:
    bool currentlySelecting;
    QGraphicsScene * scene;
    QGraphicsPixmapItem qAshImg;
    QImage Mat2QImage(const cv::Mat3b &src);
    QPoint dragStart;

};


#endif // EODIMG_H

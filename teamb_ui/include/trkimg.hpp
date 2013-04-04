#ifndef TRKIMG_H
#define TRKIMG_H

#include <QMainWindow>
#include <QtCore>
#include <QtGui>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QRubberBand>
#include <QPointF>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class trkImg : public QGraphicsView
{
    Q_OBJECT
public:
    explicit trkImg(QWidget *parent = 0);
    QRubberBand* rubber;
    void input(bool,int,int,int,int);

public Q_SLOTS:
    void updateImage(cv::Mat img);

Q_SIGNALS:

private:
    QGraphicsScene * scene;
    QGraphicsPixmapItem qAshImg;
    QImage Mat2QImage(const cv::Mat3b &src);
    QGraphicsRectItem* rect;

};


#endif // EODIMG_H

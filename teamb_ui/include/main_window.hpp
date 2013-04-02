#ifndef teamb_ui_MAIN_WINDOW_H
#define teamb_ui_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include <QFileDialog>
#include <QString>
#include <QDebug>
#include <qprocess.h>
#include <QVariant>
#include <QMessageBox>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <QPoint>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <iostream>
#include <QPoint>
#include <QPainter>
#include <QLabel>
#include <QMouseEvent>

class Controller;

class SleeperThread : public QThread
{
public:
    static void msleep(unsigned long msecs)
    {
        QThread::msleep(msecs);
    }
};

namespace teamb_ui {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    
    enum uiState{
        ui_ready,
        idle_manual,
        tracking,
        auto_nav
    };
    enum modeState{
        autoMode,
        manMode
    };

    int curr_mode;
    int prev_state;
    int curr_state;
    QTimer *timer;
    QTimer *lblTimer;

    void closeEvent(QCloseEvent *event);
    void showNoMasterMessage();
    void commandToShell(QString msg);
    void Message(const QString msg);

    int sendMode();
    int sendState();

public Q_SLOTS:
    void connectToROS();
    void toggleManualMode(int);
    void updateWindow();
    void confirmTracking();
    void resetToIdleManual();
    void setTargetInPic();
    void updateLoggingView();
    void updateNewImg(cv::Mat img);
    void updateLabel();

  Q_SIGNALS:
    void changemode(int);
    void updateRos();
    void mouseOverInfo(int,int,int,int);

private:
    cv::Mat image;
    Ui::MainWindowDesign ui;
    QNode qnode;
};

}  

#endif // trial_MAIN_WINDOW_H

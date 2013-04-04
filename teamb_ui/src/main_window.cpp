#include <QThread>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "../include/qnode.hpp"
#include "../include/main_window.hpp"
#include "../include/eodimg.hpp"
#include <stdlib.h>

namespace teamb_ui {
using namespace Qt;


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)

{
    setWindowIcon(QIcon(":/images/icon.png"));

    ui.setupUi(this);
    ui.tab_manager->setCurrentIndex(0);
    ui.view_logging->setModel(qnode.loggingModel());
    ui.pb_confirmTracking->setEnabled(false);
    ui.cb_enableManualCtrl->setEnabled(false);
    //Initialize Current State
    curr_state = ui_ready;

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(newImg(cv::Mat)), this, SLOT(updateNewImg(cv::Mat)));
    QObject::connect(&qnode, SIGNAL(trkImgDisp(cv::Mat)), this, SLOT(updateTrkImg(cv::Mat)));
    QObject::connect(&qnode, SIGNAL(coordRecvd(int,int,int,int)), this, SLOT(paintRectangle(int,int,int,int)));

    //Sending Mode change to ROS to be sent to robot
    QObject::connect(this, SIGNAL(updateRos()), &qnode, SLOT(update()));
    QObject::connect(this, SIGNAL(sendState(int)), &qnode, SLOT(publishState(int)));
    QObject::connect(this, SIGNAL(mouseOverInfo(int,int,int,int)),&qnode, SLOT(publishInfo(int,int,int,int)));

}

MainWindow::~MainWindow() {}

void MainWindow::showNoMasterMessage() {
    Msg("Couldn't find the ROS master. Please start roscore!");
    close();
}

void MainWindow::connectToROS() {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
            ui.pb_connect->setEnabled(false);
            ui.cb_enableManualCtrl->setEnabled(true);
            curr_state = idle_manual;

            timer = new QTimer(this);
            lblTimer = new QTimer(this);

            connect(timer, SIGNAL(timeout()), &qnode, SLOT(update()));
            connect(lblTimer, SIGNAL(timeout()), this, SLOT(updateLabel()));

            timer->start(10);
            lblTimer->start(10);
        }

}

void MainWindow::Msg(const QString msg){
    QMessageBox dialog;
    dialog.setText(msg);
    dialog.exec();
}

int MainWindow::MsgWithOKCancel(const QString msg){
    QMessageBox msgBox;
    msgBox.setText(msg);
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Ok);
    return msgBox.exec();
}

void MainWindow::setTargetInPic()
{
    //qDebug()<<"XW";
    //qDebug()<<ui.graphicsView->xw;
    //qDebug()<<ui.graphicsView->yw;

        if(curr_state == idle_manual){
            curr_state = tracking;
            ui.pb_setTarget->setEnabled(false);
            ui.pb_confirmTracking->setEnabled(true);
            ui.cb_enableManualCtrl->setEnabled(false);
            ui.tab_manager->setCurrentIndex(1);
        }

        const QImage tmp = (*(qnode.getCurrImg()));
        QPixmap p = QPixmap::fromImage(tmp);

        Q_EMIT mouseOverInfo(ui.graphicsView->x,ui.graphicsView->y,ui.graphicsView->xw,ui.graphicsView->yw);
        qnode.log(QNode::Info,"Target Selection Complete!");
}

void MainWindow::updateWindow()
{   Q_EMIT updateRos(); }

void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

void MainWindow::updateNewImg(cv::Mat img)
{   ui.graphicsView->resize(img.cols, img.rows);
    ui.graphicsView->updateImage(img);
}

void MainWindow::updateTrkImg(cv::Mat img)
{   ui.graphicsView_2->resize(img.cols, img.rows);
    ui.graphicsView_2->updateImage(img);

}

void MainWindow::paintRectangle(int x, int y, int xw, int yw){
    ui.graphicsView_2->input(x,y,xw,yw);
}

void MainWindow::updateLabel()
{
    switch(curr_state){
    case 0:
        ui.lbl_modeStatus->setText("UI Ready");
        break;
    case 1:
        ui.lbl_modeStatus->setText("Idle Manual");
        break;
    case 2:
        ui.lbl_modeStatus->setText("Tracking");
        break;
    case 3:
        ui.lbl_modeStatus->setText("Auto Navigation");
        break;
    }
    Q_EMIT sendState(curr_state);

}

void MainWindow::confirmTracking(){
    int ret = MsgWithOKCancel("Confirming Autonomous Travel to Marked Destination");
    if(ret == 1024)
    {
        qnode.log(QNode::Info,"Tracking confirmed.");
        ui.pb_confirmTracking->setEnabled(false);
        curr_state = auto_nav;
    }
    else{
        qnode.log(QNode::Info,"Please start mission or select manual mode.");
    }
}

void MainWindow::resetToIdleManual(){
    curr_state = idle_manual;

    qnode.log(QNode::Info,"Back to Idle/Manual State.");

    ui.cb_enableManualCtrl->setEnabled(true);
    ui.pb_confirmTracking->setEnabled(false);
    ui.pb_setTarget->setEnabled(true);
    ui.tab_manager->setCurrentIndex(0);
    ui.graphicsView->rubber->hide();
}


void MainWindow::toggleManualMode(int cb_state){

    /*Check Button
      Unchecked = 0
     *Checked   = 2 */


    if(cb_state == 0){
        ui.lbl_modeStatus->setText("Processing");
        ui.pb_setTarget->setEnabled(true);
        qnode.log(QNode::Info,"Manual control disabled.");

    }
    else{
        ui.lbl_modeStatus->setText("Processing");
        ui.pb_setTarget->setEnabled(false);
        qnode.log(QNode::Info,"Manual mode. Please Use Joystick");
        curr_state = idle_manual;
    }

}

//Function for passing command to shell
void MainWindow::commandToShell(QString msg)
{
    QProcess *process = new QProcess();
    QStringList list;
    list.append(msg);
    process->start("sh", QStringList() << "-c" << msg);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}


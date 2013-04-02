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

    lblTimer = new QTimer(this);

    ui.setupUi(this);
    ui.tab_manager->setCurrentIndex(0);
    ui.view_logging->setModel(qnode.loggingModel());
    ui.pb_confirmTracking->hide();
    ui.cb_enableManualCtrl->setEnabled(false);
    //Initialize Current State
    MainWindow::curr_mode = manMode;
    MainWindow::curr_state = ui_ready;

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(newImg(cv::Mat)), this, SLOT(updateNewImg(cv::Mat)));

    //Sending Mode change to ROS to be sent to robot
    QObject::connect(this, SIGNAL(changemode(int)), &qnode, SLOT(modeCallback(int)));
    QObject::connect(this, SIGNAL(updateRos()), &qnode, SLOT(update()));
    QObject::connect(this, SIGNAL(mouseOverInfo(int,int,int,int)),&qnode, SLOT(publishInfo(int,int,int,int)));
    connect(lblTimer, SIGNAL(timeout()), this, SLOT(updateLabel()));


    lblTimer->start(10);

    //TODO
    //Q_EMIT currMode();
}

MainWindow::~MainWindow() {}

void MainWindow::showNoMasterMessage() {
    Message("Couldn't find the ROS master. Please start roscore!");
    close();
}

void MainWindow::connectToROS() {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
            ui.pb_connect->setEnabled(false);
            ui.cb_enableManualCtrl->setEnabled(true);
            qDebug()<<curr_state<<"Current State\n";
            curr_state = idle_manual;
            qDebug()<<curr_state;
        }

 timer = new QTimer(this);
 connect(timer, SIGNAL(timeout()), &qnode, SLOT(update()));
 timer->start(10);
}

void MainWindow::Message(const QString msg){
    QMessageBox dialog;
    dialog.setText(msg);
    dialog.exec();
}

void MainWindow::setTargetInPic()
{
    if (ui.graphicsView->rubber->isVisible()){
        if(curr_state == idle_manual){
            curr_state = tracking;
            ui.pb_setTarget->setEnabled(false);
            ui.pb_confirmTracking->show();
            ui.cb_enableManualCtrl->setEnabled(false);
            ui.tab_manager->setCurrentIndex(1);
        }

        qDebug()<<"In Set Target Function";
        qDebug()<<curr_state;

        const QImage tmp = (*(qnode.getCurrImg()));
        QPixmap p = QPixmap::fromImage(tmp);
        int r = ui.graphicsView->ret;
        qDebug()<< r;
        qDebug()<<ui.graphicsView->x<<ui.graphicsView->y;
        if(ui.graphicsView->ret== 0x02000000)
        Q_EMIT mouseOverInfo(ui.graphicsView->x,ui.graphicsView->y,ui.graphicsView->xw,ui.graphicsView->yw);
    }else{
        qnode.log(QNode::Info,"Select target first!");
    }
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

void MainWindow::updateLabel()
{
    //qDebug()<<"Update Label Func";
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
}

void MainWindow::confirmTracking(){
    curr_state = auto_nav;
    qnode.log(QNode::Info,"Tracking confirmed.");
    ui.pb_confirmTracking->setEnabled(false);
    //TODO - ROSTOPIC PUBLISH - Emit Signal to publish int data
}

void MainWindow::resetToIdleManual(){
    curr_state = idle_manual;

    qnode.log(QNode::Info,"Back to Idle/Manual State.");
//    qDebug()<<"In Reset Mode, current State:"<<curr_state;
    //TODO - ROSTOPIC PUBLISH - Emit Signal to publish int data
    ui.cb_enableManualCtrl->setEnabled(true);
    ui.pb_confirmTracking->hide();
    ui.pb_setTarget->setEnabled(true);
    ui.tab_manager->setCurrentIndex(0);
    ui.graphicsView->rubber->hide();
}


void MainWindow::toggleManualMode(int cb_state){

    qDebug()<<"Toggling Manual Mode";
    //Unchecked = 0
    //Checked   = 2
    if(cb_state == 0){
        ui.lbl_modeStatus->setText("Processing");
        curr_mode = autoMode;
        ui.pb_setTarget->setEnabled(true);
        QMessageBox msgBox;
        msgBox.setText("Manual controls will be disabled.");
        msgBox.exec();
        qnode.log(QNode::Info,"Manual control disabled.");
    }
    else{
        ui.lbl_modeStatus->setText("Processing");
        curr_mode = manMode;
        ui.pb_setTarget->setEnabled(false);
        QMessageBox msgBox;
        msgBox.setText("Manual controls will be enabled.");
        msgBox.setInformativeText("Are you sure you want to go ahead?");
        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setEscapeButton(QMessageBox::Close);
        msgBox.setDefaultButton(QMessageBox::Ok);
        int ret = msgBox.exec();
        //If Ok is Pressed - 1024
        if(ret == 1024)
        {
            //MainWindow::curr_state = man_nav;
            //MainWindow::curr_mode = manMode;
            ui.lbl_modeStatus->setText("Manual Mode");

            Q_EMIT changemode(MainWindow::curr_mode);
        }
        else{
            qnode.log(QNode::Info,"Please start mission or select manual mode.");
            //ui.modeStatus->setText("Unassigned!");
        }
    }

}

int MainWindow::sendMode(){
    return MainWindow::curr_mode;
}

int MainWindow::sendState(){
    return MainWindow::curr_state;
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


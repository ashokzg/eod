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
#define MOTOR_BATTERY_VOLTAGE_HIGH 8.4
#define MOTOR_BATTERY_VOLTAGE_LOW 7.4
#define PC_BATTERY_VOLTAGE_HIGH 21
#define PC_BATTERY_VOLTAGE_LOW 18.5

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
    ui.mtrBattStatus->setRange(0,99);
    ui.pcBattStatus->setRange(0,99);
    ui.mtrBattStatus->setValue(99);
    ui.pcBattStatus->setValue(99);

    //Initialize Current State
    curr_state = ui_ready;

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(newImg(cv::Mat)), this, SLOT(updateNewImg(cv::Mat)));
    QObject::connect(&qnode, SIGNAL(trkImgDisp(cv::Mat)), this, SLOT(updateTrkImg(cv::Mat)));
    QObject::connect(&qnode, SIGNAL(coordRecvd(bool,int,int,int,int)), this, SLOT(paintRectangle(bool,int,int,int,int)));
    QObject::connect(&qnode, SIGNAL(mtrBattInfo(float)), this, SLOT(mtrBattUpdate(float)));
    QObject::connect(&qnode, SIGNAL(pcBattInfo(float)), this, SLOT(pcBattUpdate(float)));
    QObject::connect(&qnode, SIGNAL(navStateInfo(int)), this, SLOT(navStateReport(int)));
    QObject::connect(&qnode, SIGNAL(errStateInfo(int)), this, SLOT(errStateReport(int)));

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
            curr_state = idle_manual;
            Q_EMIT sendState(curr_state);

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
        if(curr_state == idle_manual){
            curr_state = tracking;
            Q_EMIT sendState(curr_state);
            ui.pb_setTarget->setEnabled(false);
            ui.pb_confirmTracking->setEnabled(true);
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

void MainWindow::paintRectangle(bool val, int x, int y, int xw, int yw){
    ui.graphicsView_2->input(val,x,y,xw,yw);
}

void MainWindow::mtrBattUpdate(float mtrBattVal) {
    float val = (mtrBattVal-MOTOR_BATTERY_VOLTAGE_LOW)*100/(MOTOR_BATTERY_VOLTAGE_HIGH-MOTOR_BATTERY_VOLTAGE_LOW);

    ui.mtrBattStatus->setValue(val);
    if(val<20){
    QMessageBox::warning(this, tr("Motor Battery Warning"),
                                   tr("The Motor Battery Supply is below it's threshold.\n"
                                      "Please change the battery and charge it!"),
                                   QMessageBox::Ok);

    }
}

void MainWindow::pcBattUpdate(float pcBattVal) {
    float val = (pcBattVal-PC_BATTERY_VOLTAGE_LOW)*100/(PC_BATTERY_VOLTAGE_HIGH-PC_BATTERY_VOLTAGE_LOW);
    ui.pcBattStatus->setValue(val);

    if(val<25){
    QMessageBox::warning(this, tr("PC Battery Warning"),
                                   tr("The PC Battery Supply is below it's threshold.\n"
                                      "Please change the battery and charge it!"),
                                   QMessageBox::Ok);
    }
}

void MainWindow::navStateReport(int state) {
    switch(state){
    case 0:
        commandToShell("rosrun sound_play say.py \"UI IS NOW READY FOR USE\"&");
        commandToShell("rosrun sound_play say.py \"UI IS NOW READY FOR USE\"&");
        commandToShell("rosrun sound_play say.py \"UI IS NOW READY FOR USE\"&");
        robot_state = 0;
        break;
    case 1:
        commandToShell("rosrun sound_play say.py \"ROBOT IS TRACKING DESTINATION. PLEASE CONFIRM THE LOCATION\"&");
        commandToShell("rosrun sound_play say.py \"ROBOT IS TRACKING DESTINATION. PLEASE CONFIRM THE LOCATION\"&");
        commandToShell("rosrun sound_play say.py \"ROBOT IS TRACKING DESTINATION. PLEASE CONFIRM THE LOCATION \"&");
        robot_state = 1;
        break;
    case 2:
        commandToShell("rosrun sound_play say.py \"ROBOT IN AUTO-MODE\"&");
        commandToShell("rosrun sound_play say.py \"ROBOT IN AUTO-MODE\"&");
        commandToShell("rosrun sound_play say.py \"ROBOT IN AUTO-MODE\"&");
        robot_state = 2;
        break;
    case 3:
        commandToShell("rosrun sound_play say.py \"ROBOT IN MANUAL MODE\"&");
        commandToShell("rosrun sound_play say.py \"ROBOT IN MANUAL MODE\"&");
        commandToShell("rosrun sound_play say.py \"ROBOT IN MANUAL MODE\"&");
        robot_state = 3;
        break;
    case 4:
        commandToShell("rosrun sound_play say.py \"ERROR!\"&");
        commandToShell("rosrun sound_play say.py \"ERROR!\"&");
        commandToShell("rosrun sound_play say.py \"ERROR!\"&");
        robot_state = 4;
        break;
    }
}

void MainWindow::errStateReport(int state) {
    switch(state){
    case 2:
        commandToShell("rosrun sound_play say.py \"DYNAMIC OBSTACLE\"&");
        commandToShell("rosrun sound_play say.py \"DYNAMIC OBSTACLE\"&");
        commandToShell("rosrun sound_play say.py \"DYNAMIC OBSTACLE\"&");
        qDebug()<<"DYNAMIC OBS";
        break;
    case 3:
        commandToShell("rosrun sound_play say.py \"UNABLE TO AVOID AFTER TRYING! PLEASE MANUALLY CONTROL\"&");
        commandToShell("rosrun sound_play say.py \"UNABLE TO AVOID AFTER TRYING! PLEASE MANUALLY CONTROL\"&");
        commandToShell("rosrun sound_play say.py \"UNABLE TO AVOID AFTER TRYING! PLEASE MANUALLY CONTROL\"&");
        qDebug()<<"UNABLE TO AVOID";
        break;
    case 4:
        commandToShell("rosrun sound_play say.py \"UNABLE TO LOCATE DESTINATION AFTER SEARCH\"&");
        commandToShell("rosrun sound_play say.py \"UNABLE TO LOCATE DESTINATION AFTER SEARCH\"&");
        commandToShell("rosrun sound_play say.py \"UNABLE TO LOCATE DESTINATION AFTER SEARCH\"&");
        qDebug()<<"UNABLE TO LOCATE DESTINATION";
        break;
    case 5:
        commandToShell("rosrun sound_play say.py \"TIME OUT OCCURRED. PLEASE MANUALLY CONTROL THE ROBOT\"&");
        commandToShell("rosrun sound_play say.py \"TIME OUT OCCURRED. PLEASE MANUALLY CONTROL THE ROBOT\"&");
        commandToShell("rosrun sound_play say.py \"TIME OUT OCCURRED. PLEASE MANUALLY CONTROL THE ROBOT\"&");
        qDebug()<<"TIME-OUT OCCURRED";
        break;
    case 6:
        commandToShell("rosrun sound_play say.py \" DESTINATION IS UNKNOWN. PLEASE LOCATE DESTINATION.\"&");
        commandToShell("rosrun sound_play say.py \" DESTINATION IS UNKNOWN. PLEASE LOCATE DESTINATION.\"&");
        commandToShell("rosrun sound_play say.py \" DESTINATION IS UNKNOWN. PLEASE LOCATE DESTINATION.\"&");
        qDebug()<<"DEST UNKNOWN";
        break;
    case 7:
        commandToShell("rosrun sound_play say.py \"ROBOT IS LOST. PLEASE MANUALLY CONTROL AND RELOCATE DESTINATION.\"&");
        commandToShell("rosrun sound_play say.py \"ROBOT IS LOST. PLEASE MANUALLY CONTROL AND RELOCATE DESTINATION.\"&");
        commandToShell("rosrun sound_play say.py \"ROBOT IS LOST. PLEASE MANUALLY CONTROL AND RELOCATE DESTINATION.\"&");
        qDebug()<<"ROBOT LOST";
        break;
    }
}

void MainWindow::updateLabel()
{
    switch(robot_state){
    case 0:
        ui.lbl_modeStatus->setText("UI READY");
        break;
    case 1:
        ui.lbl_modeStatus->setText("TRACKING DESTINATION");
        break;
    case 2:
        ui.lbl_modeStatus->setText("AUTO-MODE");
        break;
    case 3:
        ui.lbl_modeStatus->setText("MANUAL MODE");
        break;
    case 4:
        ui.lbl_modeStatus->setText("ERROR!");
    }
}

void MainWindow::confirmTracking(){
    int ret = MsgWithOKCancel("Confirming Autonomous Travel to Marked Destination");
    if(ret == 1024)
    {
        qnode.log(QNode::Info,"Tracking confirmed and sent to robot.");
        ui.pb_confirmTracking->setEnabled(false);
        curr_state = auto_nav;
        Q_EMIT sendState(curr_state);

    }
    else{
        qnode.log(QNode::Info,"Please choose destination or select manual mode.");
    }
}

void MainWindow::resetToIdleManual(){
    curr_state = idle_manual;
    Q_EMIT sendState(curr_state);

    qnode.log(QNode::Info,"Reset Pressed. Re-assign Target.");

    ui.pb_confirmTracking->setEnabled(false);
    ui.pb_setTarget->setEnabled(true);
    ui.tab_manager->setCurrentIndex(0);
    ui.graphicsView->rubber->hide();
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


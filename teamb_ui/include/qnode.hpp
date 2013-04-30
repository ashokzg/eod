#ifndef trial_QNODE_HPP_
#define trial_QNODE_HPP_

#include <QThread>
#include <QStringListModel>
#include <QMessageBox>
#include <QtGui>
#include <QApplication>

#include <ros/ros.h>
#include <ros/network.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <iostream>
#include <sstream>

#include "../msg_gen/cpp/include/teamb_ui/Dest.h"
#include "../msg_gen/cpp/include/teamb_ui/Status.h"
//#include "main_window.hpp"

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

namespace teamb_ui {

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

    enum LogLevel {
             Debug,
             Info,
             Warn,
             Error,
             Fatal
     };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
	void modefbFromRobot(const std_msgs::String::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
    void shutdown();
	QImage* getCurrImg();
    void dispCoord(const Dest ImgAreaRecvd);
    void mtrBattInput(const std_msgs::Float32 input);
    void pcBattInput(const std_msgs::Float32 input);
    void dispRobotState(const std_msgs::UInt32 state);
    void dispErrInput(const std_msgs::UInt32 state);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void updateMode();
    void newImg(cv::Mat img);
    void trkImgDisp(cv::Mat img);
    void coordRecvd(bool,int,int,int,int);
    void mtrBattInfo(float);
    void pcBattInfo(float);
    void navStateInfo(int);
    void errStateInfo(int);


public Q_SLOTS:
	void update();
    void publishInfo(int,int,int,int);
    void publishState(int);
    
private:
	int init_argc;
	char** init_argv;
    ros::Publisher dest_msg;
    ros::Publisher mode_msg;
    ros::Publisher uiStatus_msg;

    ros::Subscriber rect_msg;
    ros::Subscriber mtr_batt;
    ros::Subscriber pc_batt;
    ros::Subscriber nav_state;
    ros::Subscriber err_state;

    image_transport::Subscriber selImage;
    image_transport::Subscriber trkImage;

    QImage *currImg;
	QStringListModel logging_model;
    Dest ImgAreaSelected;
    std_msgs::Int32 rosState;
};

}  // namespace trial

#endif /* trial_QNODE_HPP_ */

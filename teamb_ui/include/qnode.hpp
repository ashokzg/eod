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
	void timercallback(const ros::TimerEvent&);
    void shutdown();
	QImage* getCurrImg();


Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void imgUpdated();
    void newImg(cv::Mat img);
    void newImg2(cv::Mat img);

public Q_SLOTS:
    void modeCallback(int);
	void update();
    void publishInfo(int,int,int,int);
    
private:
	int init_argc;
	char** init_argv;
    ros::Publisher dest_msg;
    ros::Publisher info;
    ros::Publisher status;
    ros::Subscriber robotStatus;
	image_transport::Subscriber sub;
	image_transport::Subscriber sub2;
	QImage *currImg; 
	QStringListModel logging_model;
};

}  // namespace trial

#endif /* trial_QNODE_HPP_ */

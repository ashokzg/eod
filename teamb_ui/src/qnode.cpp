#include "../include/qnode.hpp"
#include "eodimg.hpp"

namespace teamb_ui {

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{
     currImg = new QImage();
	}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
    //ros::init(init_argc,init_argv,"foo");
	if ( ! ros::master::check() ) {
		return false;
	}
    ros::start();
	ros::NodeHandle n;

    mode_msg = n.advertise<std_msgs::Int32>("/Mode", 1000);
    uiStatus_msg = n.advertise<std_msgs::Int32>("/UiStatus", 1000);
    dest_msg = n.advertise<Dest>("/UserDestination", 1000);

    image_transport::ImageTransport it(n);
    sub = it.subscribe("/camera/image_raw", 1, &QNode::imageCallback,this);
//   sub2 = it.subscribe("/camera/image_raw", 1, &QNode::imageCallback2,this);
    sub2 = it.subscribe("Tracked_Destination", 1, &QNode::imageCallback2,this);
    rect_msg = n.subscribe("/destination", 1, &QNode::sendCoord,this);
    start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	if ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;
        ss << "System Check";
		msg.data = ss.str();
        log(Info,std::string("Connected to ROS: ")+msg.data);
		loop_rate.sleep();
	}
    else
		shutdown();
}

void QNode::publishInfo(int x, int y, int xw, int yw){
    ImgAreaSelected.destPresent = true;
    ImgAreaSelected.destX= x;
    ImgAreaSelected.destY= y;
    ImgAreaSelected.destWidth= xw;
    ImgAreaSelected.destHeight= yw;
    dest_msg.publish(ImgAreaSelected);
}

void QNode::sendCoord(const Dest ImgAreaRecvd){
        bool destPresent;
        int x,y,xw,yw;
        destPresent = ImgAreaRecvd.destPresent;
        x = ImgAreaRecvd.destX;
        y = ImgAreaRecvd.destY;
        xw = ImgAreaRecvd.destWidth;
        yw = ImgAreaRecvd.destHeight;
        qDebug()<<"Img Coord Recvd"<<x<<y<<xw<<yw;
        Q_EMIT coordRecvd(destPresent,x,y,xw,yw);
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;

	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG]" << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
                logging_model_msg << "[INFO]" << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
                logging_model_msg << "[WARNING]" << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
                logging_model_msg << "[ERROR]" << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
                logging_model_msg << "[FATAL]" << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit loggingUpdated();
}

void QNode::shutdown()
{
std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
Q_EMIT rosShutdown();
}


void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std_msgs::String info;
    std::stringstream ss;

    cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ss << "cv_bridge exception: %s"<<e.what();
            info.data = ss.str();
            log(Error,info.data.c_str());

            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        QImage *qImage = new QImage(
                    cv_ptr->image.data,
                    cv_ptr->image.size().width,
                    cv_ptr->image.size().height,
                    QImage::Format_RGB888
                );
        (*currImg) = (*qImage).rgbSwapped();

        Q_EMIT newImg(cv_ptr->image);
}

void QNode::imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
    std_msgs::String info;
    std::stringstream ss;

    cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ss << "cv_bridge exception: %s"<<e.what();
            info.data = ss.str();
            log(Error,info.data.c_str());

            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        QImage *qImage = new QImage(
                    cv_ptr->image.data,
                    cv_ptr->image.size().width,
                    cv_ptr->image.size().height,
                    QImage::Format_RGB888
                );
        (*currImg) = (*qImage).rgbSwapped();

        Q_EMIT trkImgDisp(cv_ptr->image);

}

QImage* QNode::getCurrImg(){
    return currImg;
}


void QNode::publishState(int state)
{
    rosState.data = state;
    uiStatus_msg.publish(rosState);
}

void QNode::update()
{
    ros::spinOnce();
}


void QNode::modefbFromRobot(const std_msgs::String::ConstPtr& msg)
{
   ROS_INFO("I heard: [%s]", msg->data.c_str());
   log(Info,msg->data.c_str());
}

}

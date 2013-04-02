/*
 * RosTLDWrapper.cpp
 *
 *  Created on: Mar 19, 2013
 *      Author: ashok
 */

#include "OpenTLD.h"
#include "ros/ros.h"
//#include <OpenTLD/Dest.h>
#include "../../msg_gen/cpp/include/OpenTLD/Dest.h"  //This is a hack. Replace with proper header

//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>


OpenTLD::Dest destination;
//Initialize the destination location publisher
ros::Publisher destPub;

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("RosTLDWrapper::imageCallback::cv_bridge exception: %s", e.what());
        return;
    }

    printf("Obtained image");
    //cv_ptr->image;

}


int main(int argc, char **argv)
{
	//Initialize the ROS node
	ros::init(argc, argv, "TLD");
	ros::NodeHandle nh;


	printf("Program started\n");


	ros::Rate loop_rate(10);
	destPub = nh.advertise<OpenTLD::Dest>("destination", 1000);
	destination.destPresent = 0;
	destination.destX = 0;
	destination.destY = 0;
	destination.destWidth = 0;
	destination.destHeight = 0;
	destPub.publish(destination);
	ros::spinOnce();


	otld_main(argc, argv);

	//Initialize the image transport subscriber
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imgSub = it.subscribe("camera/image_raw", 1, imageCallback);

	loop_rate.sleep();


}




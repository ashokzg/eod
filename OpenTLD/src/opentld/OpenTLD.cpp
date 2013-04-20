/*  Copyright 2011 AIT Austrian Institute of Technology
*
*   This file is part of OpenTLD.
*
*   OpenTLD is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   OpenTLD is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with OpenTLD.  If not, see <http://www.gnu.org/licenses/>.
*
*/

/**
  * @author Georg Nebehay
  */

#include "Main.h"
#include "Config.h"
#include "ImAcq.h"
#include "Gui.h"


using tld::Config;
using tld::Gui;
using tld::Settings;

#include "OpenTLD.h"
#include "ros/ros.h"
#include <OpenTLD/Dest.h>
//#include "../../msg_gen/cpp/include/OpenTLD/Dest.h"  //This is a hack. Replace with proper header

//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/UInt32.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>


OpenTLD::Dest destination, userDest;

//Initialize the destination location publisher
ros::Publisher destPub;


static bool tldInit = false;
static int navState = 0; //Corresponds to IDLE state

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
Main *mainTld;


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

    if(userDest.destPresent == true)
    {
    	ROS_INFO("Tracking INIT");
    	mainTld->doWork(userDest, cv_ptr->image);
    	tldInit = true;
    	userDest.destPresent = false;
    }
    //Do not process until it is initialized atleast once. Navigation must be in either tracking or autonomous mode
    else if(tldInit == true && ((navState == 1) || (navState == 2))) 
    {
    	destination = mainTld->destTrack(&(cv_ptr->image));
    }
    
    if(navState == 1 || navState == 2)
    {
      //Publish the destination rectange
      destPub.publish(destination);
    }
    else
    {
      //Publish default value
      destination.destPresent = 0;
      destination.destX = 0;
      destination.destY = 0;
      destination.destWidth = 0;
      destination.destHeight = 0;
      destPub.publish(destination);        
    }
}

void userSelectedDestination(const OpenTLD::Dest::ConstPtr& d)
{
  ROS_INFO("User selected destination, %d", d->destPresent);
  userDest.destPresent = d->destPresent;
  userDest.destX = d->destX;
  userDest.destY = d->destY;
  userDest.destHeight = d->destHeight;
  userDest.destWidth = d->destWidth;
}

void navStateCb(const std_msgs::UInt32::ConstPtr& s)
{
  navState = s->data;  
}

int main(int argc, char **argv)
{
	//Initialize the ROS node
	ros::init(argc, argv, "TLD");
	ros::NodeHandle nh;


	printf("Program started\n");


	ros::Rate loop_rate(30);
	destPub = nh.advertise<OpenTLD::Dest>("destination", 1000);
	destination.destPresent = 0;
	destination.destX = 0;
	destination.destY = 0;
	destination.destWidth = 0;
	destination.destHeight = 0;
	destPub.publish(destination);
	ros::spinOnce();
	std::string camName;
  nh.param<std::string>("/eod_cam", camName, "camera/image_raw");  
  userDest.destPresent = false;

	//Initialize the image transport subscriber
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imgSub = it.subscribe(camName, 1, imageCallback);

	ros::Subscriber userDestSub = nh.subscribe("/UserDestination", 1000, userSelectedDestination);
	ros::Subscriber navStateSub = nh.subscribe("/Nav_State", 1000, navStateCb);

	/*---------------------------------------------------------
	 *    START OF TLD
	 *--------------------------------------------------------*/
	mainTld = new Main();
	Config config;

	ROS_WARN("Start of TLD\n");


	if(config.init(argc, argv) == PROGRAM_EXIT)
	{
		ROS_ERROR("TLD Unfortunate Exit");
		return EXIT_FAILURE;
	}

	config.configure(mainTld);

	srand(mainTld->seed);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return EXIT_SUCCESS;
}

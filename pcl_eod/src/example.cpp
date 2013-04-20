#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	ROS_INFO("PCL Processing Started");
	sensor_msgs::PointCloud2 output;
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg (*input, *cloudPtr);
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloudPtr);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);
	pcl::toROSMsg(*cloud_filtered, output);
	pub.publish(output);
	ROS_INFO("PCL Processing Ended");

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_eod");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/eodCam/points2", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  // Spin
  ros::spin ();
}

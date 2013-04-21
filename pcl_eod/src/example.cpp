#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_eod/Clusters.h>
#include <math.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <boost/thread/thread.hpp>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


ros::Publisher pub;
ros::Publisher clusterPub;
//pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_removed16(new pcl16::PointCloud<pcl16::PointXYZ>);

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

	ROS_INFO("PCL Started");
	ROS_INFO("PCL Processing Started");
	sensor_msgs::PointCloud2 output;
	pcl_eod::Clusters cluster;
	pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg (*input, *cloudPtr);
	//-----------------
	for(unsigned int i = 0; i < cloudPtr->size(); i += 4)
	{
		if(isnan(cloudPtr->points[i].z) == false)
		{
			if(cloudPtr->points[i].z > 5.5)
			{
				cloudPtr->points[i].x = NAN;
				cloudPtr->points[i].y = NAN;
				cloudPtr->points[i].z = NAN;
			}
			else
			{
				cloud_removed->push_back(cloudPtr->points[i]);
			}
		}
	}
	  std::vector<int> inliers;
	  // created RandomSampleConsensus object and compute the appropriated model
	  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr   model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_removed));

	    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	    ransac.setDistanceThreshold (.01);
	    ransac.computeModel();
	    ransac.getInliers(inliers);

	    for( std::vector<int>::const_iterator i = inliers.begin(); i != inliers.end(); ++i)
	        std::cout << *i << ' ';

	  // copies all inliers of the model computed to another PointCloud
	  pcl::copyPointCloud<pcl::PointXYZ>(*cloud_removed, inliers, *final);


	//pcl::toROSMsg(*cloud_cluster, output);
	pcl::toROSMsg(*final, output);
	output.header.frame_id = "camera";
	pub.publish(output);
	clusterPub.publish(cluster);
	ROS_INFO("PCL Ended");
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
  clusterPub = nh.advertise<pcl_eod::Clusters>("clusters", 5);
  // Spin
  ros::spin ();
}

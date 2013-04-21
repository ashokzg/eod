#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_eod/Clusters.h>
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

ros::Publisher pub;
ros::Publisher clusterPub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

	ROS_INFO("PCL Started");
	ROS_INFO("PCL Processing Started");
	sensor_msgs::PointCloud2 output;
	pcl_eod::Clusters cluster;
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg (*input, *cloudPtr);


	//-----------------

	  // Create the filtering object: downsample the dataset using a leaf size of 1cm
	  pcl::VoxelGrid<pcl::PointXYZ> vg;
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	  vg.setInputCloud (cloudPtr);
	  vg.setLeafSize (0.03f, 0.03f, 0.03f);
	  vg.filter (*cloud_filtered);
	 // std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

	  // Create the segmentation object for the planar model and set all the parameters
	  pcl::SACSegmentation<pcl::PointXYZ> seg;
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
	  pcl::PCDWriter writer;
	  seg.setOptimizeCoefficients (true);
	  seg.setModelType (pcl::SACMODEL_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (100);
	  seg.setDistanceThreshold (0.10);

	  int nr_points = (int) cloud_filtered->points.size ();
	  while (cloud_filtered->points.size () > 0.3 * nr_points)
	  {
	    // Segment the largest planar component from the remaining cloud
	    seg.setInputCloud (cloud_filtered);
	    seg.segment (*inliers, *coefficients);
	    if (inliers->indices.size () == 0)
	    {
	      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	      break;
	    }

	    // Extract the planar inliers from the input cloud
	    pcl::ExtractIndices<pcl::PointXYZ> extract;
	    extract.setInputCloud (cloud_filtered);
	    extract.setIndices(inliers);
	    extract.setNegative (false);

	    // Write the planar inliers to disk
	    extract.filter (*cloud_plane);
	    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

	    // Remove the planar inliers, extract the rest
	    extract.setNegative (true);
	    extract.filter (*cloud_f);
	    *cloud_filtered = *cloud_f;
	  }
	  //ROS_INFO("%d %d %d", cloud_filtered->width, cloud_filtered->height, cloud_filtered->size());
	  if(cloud_filtered->size() <= 0)
	  {
		  ROS_ERROR("Size 0 point cloud received");
		  return ;
	  }
	  // Creating the KdTree object for the search method of the extraction
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud(cloud_filtered);

	  std::vector<pcl::PointIndices> cluster_indices;
	  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	  ec.setClusterTolerance (0.1); // 10cm
	  ec.setMinClusterSize (10);
	  ec.setMaxClusterSize (25000);
	  ec.setSearchMethod (tree);
	  ec.setInputCloud (cloud_filtered);
	  ec.extract (cluster_indices);

	  int j = 0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointIndices clusterIdx;

	  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
		geometry_msgs::Point p;
		p.z = 1000;
	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	    {
	      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
	      if(cloud_filtered->points[*pit].z < p.z)
	      {
	    	  p.x = cloud_filtered->points[*pit].x;
	    	  p.y = cloud_filtered->points[*pit].y;
	    	  p.z = cloud_filtered->points[*pit].z;
	    	  clusterIdx = *it;
	      }
	    }
       cluster.point.push_back(p);

	    //std::cout << "Minimum dist point is for this cluster is at " << minDistPt[0] << ", " << minDistPt[1] << ", " << minDistPt[2] << std::endl;
	    ROS_INFO("Minimum dist point is for this cluster is at %f, %f, %f", p.x, p.y, p.z);
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;
	    j++;
	  }
	cluster.header.frame_id = "/camera";
	cluster.header.stamp = ros::Time::now();


	pcl::toROSMsg(*cloud_cluster, output);
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

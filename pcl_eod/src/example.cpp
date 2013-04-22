#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_eod/Clusters.h>
#include <math.h>
// PCL specific includes
#include <pcl16/ros/conversions.h>
#include <pcl16/point_cloud.h>
#include <pcl16/point_types.h>
#include <pcl16/sample_consensus/model_types.h>
#include <pcl16/sample_consensus/method_types.h>
#include <pcl16/segmentation/sac_segmentation.h>
#include <pcl16/filters/statistical_outlier_removal.h>
#include <pcl16/io/pcd_io.h>
#include <pcl16/ModelCoefficients.h>
#include <pcl16/filters/extract_indices.h>
#include <pcl16/filters/voxel_grid.h>
#include <pcl16/features/normal_3d.h>
#include <pcl16/kdtree/kdtree.h>
#include <pcl16/segmentation/extract_clusters.h>
#include <pcl16/filters/extract_indices.h>
#include <pcl16/sample_consensus/ransac.h>
#include <pcl16/sample_consensus/sac_model_plane.h>
#include <pcl16/sample_consensus/sac_model_sphere.h>
#include <boost/thread/thread.hpp>

#include <pcl16/console/parse.h>
#include <pcl16/filters/extract_indices.h>
#include <pcl16/io/pcd_io.h>
#include <pcl16/point_types.h>
#include <pcl16/sample_consensus/ransac.h>
#include <pcl16/sample_consensus/sac_model_plane.h>
#include <pcl16/sample_consensus/sac_model_sphere.h>
#include <pcl16/sample_consensus/sac_model_normal_parallel_plane.h>
//#include <pcl16/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl16/segmentation/region_3d.h>


ros::Publisher pub;
ros::Publisher pub1;
ros::Publisher clusterPub;
//pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_removed16(new pcl16::PointCloud<pcl16::PointXYZ>);

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

	ROS_INFO("PCL Started");
	ROS_INFO("PCL Processing Started");
	sensor_msgs::PointCloud2 output, output1;
	pcl_eod::Clusters cluster;
	pcl16::PointCloud<pcl16::PointXYZ>::Ptr final (new pcl16::PointCloud<pcl16::PointXYZ>);
	pcl16::PointCloud<pcl16::PointXYZ>::Ptr final1 (new pcl16::PointCloud<pcl16::PointXYZ>);
	// Convert the sensor_msgs/PointCloud2 data to pcl16/PointCloud
	pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloudPtr(new pcl16::PointCloud<pcl16::PointXYZ>), cloud_f (new pcl16::PointCloud<pcl16::PointXYZ>);

	pcl16::PointCloud<pcl16::PointXYZ>::Ptr  cloud_removed(new pcl16::PointCloud<pcl16::PointXYZ>);
	pcl16::fromROSMsg (*input, *cloudPtr);
	//cloud_removed.resize(5);
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
	for(int i = 0; i < 2; i++)
	{

	  if(cloud_removed->size() > 0)
	  {

		std::vector<int> inliers;
		// created RandomSampleConsensus object and compute the appropriated model
		pcl16::SampleConsensusModelPlane<pcl16::PointXYZ>::Ptr   model_p (new pcl16::SampleConsensusModelPlane<pcl16::PointXYZ> (cloud_removed));

		pcl16::SampleConsensusModelNormalParallelPlane<pcl16::PointXYZ , pcl16::Normal>::Ptr model_pn (new pcl16::SampleConsensusModelNormalParallelPlane<pcl16::PointXYZ, pcl16::Normal> (cloud_removed));
		Eigen::Vector3f n(1,0,0);
		model_pn->setAxis(n);
		model_pn->setEpsAngle(3.14);
		model_pn->setDistanceFromOrigin(1.5);
		model_pn->setEpsDist(4);


		pcl16::RandomSampleConsensus<pcl16::PointXYZ> ransac (model_p);


		ransac.setDistanceThreshold (0.05);
		ransac.computeModel();
		ransac.getInliers(inliers);

		//	    for( std::vector<int>::const_iterator i = inliers.begin(); i != inliers.end(); ++i)
		//	        std::cout << *i << ' ';

		// copies all inliers of the model computed to another PointCloud
		if(i == 0)
			pcl16::copyPointCloud<pcl16::PointXYZ>(*cloud_removed, inliers, *final);
		else
			pcl16::copyPointCloud<pcl16::PointXYZ>(*cloud_removed, inliers, *final1);
		std::cout<<cloud_removed->size()<<" ";
		for( std::vector<int>::const_iterator k = inliers.begin(); k != inliers.end(); k++)
		{
		  cloud_removed->erase(cloud_removed->begin()+(*k));
		}
		std::cout<<cloud_removed->size()<<" ";

	  }
	  else
		  break;
	}
	//pcl16::toROSMsg(*cloud_cluster, output);
	pcl16::toROSMsg(*final, output);
	pcl16::toROSMsg(*final1, output1);
	output.header.frame_id = "camera";
	output1.header.frame_id = "camera";
	pub.publish(output);
	pub1.publish(output1);
	//clusterPub.publish(cluster);

	ROS_INFO("PCL Ended");
	ROS_INFO("PCL Processing Ended");
}


void cb(const sensor_msgs::PointCloud2ConstPtr& input)
{

	ROS_INFO("PCL Started");
	ROS_INFO("PCL Processing Started");
	sensor_msgs::PointCloud2 output;
	pcl_eod::Clusters cluster;
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloudPtr(new pcl16::PointCloud<pcl16::PointXYZ>), cloud_f (new pcl16::PointCloud<pcl16::PointXYZ>);
	pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_removed(new pcl16::PointCloud<pcl16::PointXYZ>);
	pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_final(new pcl16::PointCloud<pcl16::PointXYZ>);
	pcl16::fromROSMsg (*input, *cloudPtr);
	//-----------------


	  // Create the filtering object: downsample the dataset using a leaf size of 1cm
	  pcl16::VoxelGrid<pcl16::PointXYZ> vg;
	  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_filtered (new pcl16::PointCloud<pcl16::PointXYZ>);
	  vg.setInputCloud (cloudPtr);
	  vg.setLeafSize (0.05f, 0.05f, 0.05f);
	  vg.filter (*cloud_removed);


	  // Create the segmentation object for the planar model and set all the parameters
//	  pcl16::SACSegmentation<pcl16::PointXYZ> seg;
//	  pcl16::PointIndices::Ptr inliers (new pcl16::PointIndices);
//	  pcl16::ModelCoefficients::Ptr coefficients (new pcl16::ModelCoefficients);
//	  pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_plane (new pcl16::PointCloud<pcl16::PointXYZ> ());
//	  pcl16::PCDWriter writer;
//	  seg.setOptimizeCoefficients (true);
//	  seg.setModelType (pcl16::SACMODEL_PLANE);
//	  seg.setMethodType (pcl16::SAC_RANSAC);
//	  seg.setMaxIterations (10);
//	  seg.setDistanceThreshold (0.10);

//	  int nr_points = (int) cloud_removed->points.size ();
//	  while (cloud_removed->points.size () > 0.3 * nr_points)
//	  {
//	    // Segment the largest planar component from the remaining cloud
//	    seg.setInputCloud (cloud_removed);
//	    seg.segment (*inliers, *coefficients);
//	    if (inliers->indices.size () == 0)
//	    {
//	      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//	      break;
//	    }
//
//	    // Extract the planar inliers from the input cloud
//	    pcl16::ExtractIndices<pcl16::PointXYZ> extract;
//	    extract.setInputCloud (cloud_removed);
//	    extract.setIndices(inliers);
//	    extract.setNegative (false);
//
//	    // Write the planar inliers to disk
//	    extract.filter (*cloud_plane);
//	    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
//
//	    // Remove the planar inliers, extract the rest
//	    extract.setNegative (true);
//	    extract.filter (*cloud_f);
//	    *cloud_removed = *cloud_f;
//	  }
	  //ROS_INFO("%d %d %d", cloud_filtered->width, cloud_filtered->height, cloud_filtered->size());
	  if(cloud_removed->size() <= 0)
	  {
		  ROS_ERROR("Size 0 point cloud received");
		  return ;
	  }
	  // Creating the KdTree object for the search method of the extraction
	  pcl16::search::KdTree<pcl16::PointXYZ>::Ptr tree (new pcl16::search::KdTree<pcl16::PointXYZ>);
	  tree->setInputCloud(cloud_removed);

	  std::vector<pcl16::PointIndices> cluster_indices;
	  pcl16::EuclideanClusterExtraction<pcl16::PointXYZ> ec;

	  ec.setClusterTolerance (0.1); // 10cm
	  ec.setMinClusterSize (50);
	  ec.setMaxClusterSize (2500);
	  ec.setSearchMethod (tree);
	  ec.setInputCloud (cloud_removed);
	  ec.extract (cluster_indices);

	  int j = 0;
	  float minDist = 1000, maxDist = 0, t;
      pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_cluster (new pcl16::PointCloud<pcl16::PointXYZ>);
      pcl16::PointIndices clusterIdx;

	  for (std::vector<pcl16::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
		geometry_msgs::Point p, minP, maxP;
		p.x = 0;
		p.y = 0;
		p.z = 0;
		int count = 0;
		minDist = 1000;
		maxDist = 0;
	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	    {
	    	  p.x += cloud_removed->points[*pit].x;
	    	  p.y += cloud_removed->points[*pit].y;
	    	  p.z += cloud_removed->points[*pit].z;
	    	  clusterIdx = *it;
	    	  count++;
	    }
	    p.x /= count;
	    p.y /= count;
	    p.z /= count;
	    //std::cout << "Minimum dist point is for this cluster is at " << minDistPt[0] << ", " << minDistPt[1] << ", " << minDistPt[2] << std::endl;
	    ROS_INFO("Avg. dist for this cluster is at %f, %f, %f", p.x, p.y, p.z);
	    if(p.z < 5 && p.y > -0.7 )//&& p.y < 0.15)
	    {
	    	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	    	{
	    		cloud_removed->points[*pit].z = p.z;
	    		cloud_removed->points[*pit].y = 0.2;
		    	cloud_cluster->points.push_back (cloud_removed->points[*pit]); //*
				t = sqrt(cloud_removed->points[*pit].x*cloud_removed->points[*pit].x
					  + cloud_removed->points[*pit].y*cloud_removed->points[*pit].y
					  + cloud_removed->points[*pit].z*cloud_removed->points[*pit].z);
				if(minDist > t)
				{
				  minDist = t;
				  minP.x = cloud_removed->points[*pit].x;
				  minP.y = cloud_removed->points[*pit].y;
				  minP.z = cloud_removed->points[*pit].z;
				}
				if(maxDist < t)
				{
					maxDist = t;
					maxP.x = cloud_removed->points[*pit].x;
					maxP.y = cloud_removed->points[*pit].y;
					maxP.z = cloud_removed->points[*pit].z;
				}
	    	}
		    ROS_INFO("Min dist for this cluster is at %f, %f, %f", minP.x, minP.y, minP.z);
		    ROS_INFO("Max dist for this cluster is at %f, %f, %f", maxP.x, maxP.y, maxP.z);
	    	cluster.minpoint.push_back(minP);
	    	cluster.maxpoint.push_back(maxP);

	    	ROS_INFO("Cluster added");
	    }
	    else
	    	ROS_INFO("REJECTED");

	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;
	    j++;
	}
	cluster.header.frame_id = "/camera";
	cluster.header.stamp = ros::Time::now();

//	for(unsigned int i = 0; i < cloud_cluster->size(); i += 1)
//	{
//		if(isnan(cloud_cluster->points[i].z) == false)
//		{
//			if(cloud_cluster->points[i].z < 5.5)
//			{
//				if(cloud_cluster->points[i].y < 0.25 && cloud_cluster->points[i].y > -0.7)
//				{
//					cloud_final->push_back(cloud_cluster->points[i]);
//				}
//			}
//		}
//	}
	std::cout<<std::endl;
	pcl16::toROSMsg(*cloud_cluster, output);
	//pcl16::toROSMsg(*cloud_final, output);
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
  ros::Subscriber sub = nh.subscribe ("/eodCam/points2", 1, cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output1", 1);
  clusterPub = nh.advertise<pcl_eod::Clusters>("clusters", 5);
  // Spin
  ros::spin ();
}

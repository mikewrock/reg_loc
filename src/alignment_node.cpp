#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../include/registration_localization/alignment_node.hpp"
#include <sstream>
#include <ros/network.h>
#include <string>
#include <stdlib.h> 


// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <pcl/conversions.h>
#include "pcl_ros/transforms.h"
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include "std_msgs/String.h"
#include <sstream>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/JointState.h>  
#include <pcl/octree/octree.h>
#include <vector>
#include <ctime>
#include <boost/make_shared.hpp>
#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
namespace registration_localization {

namespace align{
/*****************************************************************************
** Implementation
*****************************************************************************/



QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}


bool QNode::init(std::string topic, std::string name, std::string location) {

	int fargc = 0;
	char** fargv = (char**)malloc(sizeof(char*)*(fargc+1));
	ros::init(fargc,fargv,"alignNode");
	free(fargv);
	if ( ! ros::master::check() ) {
		return false;

	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.

	ros::NodeHandle n;
	// Add your ros communications here.

	// Create a ROS subscriber for the input point cloud
	std::stringstream topicss;
	topicss << topic << "_aligned";
	std::string full_topic = topicss.str();
	ROS_INFO_STREAM("Broadcasting aligned cloud on /" << full_topic);

	aligned_pub = n.advertise<sensor_msgs::PointCloud2> (full_topic, 1);
	marker_pub = n.advertise<sensor_msgs::PointCloud2> ("marker_topic", 1);
	marker_aligned_pub = n.advertise<sensor_msgs::PointCloud2> ("aligned_marker_topic", 1);
	
	// Create a ROS subscriber for the input point cloud
	cloud_sub = n.subscribe (topic, 1, &QNode::cloud_cb, this);

	//load marker file
	std::stringstream ss;
	ss << location << name << ".pcd";
	filename.data = ss.str();
	if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename.data.c_str(), *marker_cloud) == -1) 
    	{
		PCL_ERROR ("Couldn't find marker file \n");
	}else{

		ROS_INFO("Marker Loaded");
	}

	start();
	return true;
}

void QNode::run() {
	while ( ros::ok() ) {
		ros::spinOnce();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}



void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource (cloud_src);
  icp.setInputTarget (cloud_tgt);
  typedef pcl::registration::TransformationEstimationLM <pcl::PointXYZ, pcl::PointXYZ> te;
  boost::shared_ptr<te> teLM (new te);
  icp.setTransformationEstimation (teLM);
  icp.setMaximumIterations(100);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align (Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
if(icp.getFitnessScore() > 0.01){
ROS_INFO("trying again");  
icp.setTransformationEstimation (teLM);
  icp.setMaximumIterations(500);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align (Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
}
  std::cout << icp.getFinalTransformation() << std::endl;
  Eigen::Matrix4f T;
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  T = icp.getFinalTransformation();
  // Get the transformation from target to source
  targetToSource = T.inverse();
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);
  final_transform = targetToSource;
}



void QNode::align() {

	//Create containers for filtered clouds
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZI> pass (true);
	//filter the data
	  pass.setInputCloud (cloud);
	  pass.setFilterFieldName ("intensity");
	  pass.setFilterLimits (800,5000);
	  pass.filter (*cloud_filtered);
		//Align the two clouds
	
	  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZI> );
	  pcl::PointCloud<pcl::PointXYZI>::Ptr marker_cloud_aligned (new pcl::PointCloud<pcl::PointXYZI> );
	PointCloud::Ptr result (new PointCloud), source (new PointCloud), target (new PointCloud);
	  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

	  // Fill in the source cloud data
	  source->width  = marker_cloud->width;
	  source->height = marker_cloud->height;
	  source->points.resize (marker_cloud->width * marker_cloud->height);
	  for (size_t i = 0; i < source->points.size (); ++i)
	  {
	    source->points[i].x = marker_cloud->points[i].x;
	    source->points[i].y = marker_cloud->points[i].y;
	    source->points[i].z = marker_cloud->points[i].z;
	  }

	  // Fill in the target cloud data
	  target->width  = cloud_filtered->width;
	  target->height = cloud_filtered->height;
	  target->points.resize (cloud_filtered->width * cloud_filtered->height);

	  for (size_t i = 0; i < target->points.size (); ++i)
	  {
	    target->points[i].x = cloud_filtered->points[i].x;
	    target->points[i].y = cloud_filtered->points[i].y;
	    target->points[i].z = cloud_filtered->points[i].z;
	  }

	  //do the actual aligning
	  PointCloud::Ptr temp (new PointCloud);
	  pairAlign (source, target, temp, pairTransform, false);
	  pcl::transformPointCloud (*cloud, *cloud_aligned, pairTransform);
	  pcl::transformPointCloud (*cloud_filtered, *marker_cloud_aligned, pairTransform);

	  // Convert to ROS data type
	  sensor_msgs::PointCloud2 output;
	  pcl::toROSMsg(*cloud_aligned, output);
	  output.header.frame_id = "base_link";
	  output.header.stamp = ros::Time::now();
	  aligned_pub.publish(output);

	  // Convert to ROS data type
	  sensor_msgs::PointCloud2 marker_output;
	  pcl::toROSMsg(*marker_cloud_aligned, marker_output);
	  output.header.frame_id = "base_link";
	  output.header.stamp = ros::Time::now();
	  marker_aligned_pub.publish(marker_output);

}

void QNode::showMarker() {
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*marker_cloud, output);
	  output.header.frame_id = "base_link";
	  output.header.stamp = ros::Time::now();
	marker_pub.publish(output);
ROS_INFO("Marker Published");


}

void QNode::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  
ROS_INFO("GOT A CLOUD!");
sensor_msgs::PointCloud2 cloud_msg2 = *cloud_msg;

//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
cloud_msg2.fields[3].name = "intensity";
//Create a PCL pointcloud
//Populate the PCL pointcloud with the ROS message
pcl::fromROSMsg(cloud_msg2,*cloud);



  // Convert to ROS data type
 /* sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  pcl::io::savePCDFileASCII (filename.data.c_str(), *cloud_filtered);
ROS_INFO("Saved to %s", filename.data.c_str());
*/
}
}

}

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
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
#include <pcl/octree/octree.h>
#include <vector>
#include <ctime>
#include <boost/make_shared.hpp>
#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <limits>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <dynamic_reconfigure/server.h>
#include <registration_localization/SEGConfig.h>
/*

k_search
max_iterations
radius_search
distance_threshold
normal_distance_weight
eps_angle
leaf_size
optimize_coefficients
*/

  ros::Publisher pub;
  ros::Publisher pub1;
  ros::Publisher pub2;
  ros::Publisher pub3;
  ros::Publisher pub4;
  ros::Publisher pub5;
  ros::Publisher pub6;

float leaf_size;
int k_search;
int max_iterations;
float radius_search;
float distance_threshold;
float normal_distance_weight;
float distance_threshold2;
float normal_distance_weight2;
float eps_angle;
bool optimize_coefficients;
bool optimize_coefficients2;
bool reco, downsample;

void callback(registration_localization::SEGConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request:");

reco = true;
k_search = config.k_search;
max_iterations = config.max_iterations;
radius_search = config.radius_search;
distance_threshold = config.distance_threshold;
normal_distance_weight = config.normal_distance_weight;
distance_threshold2 = config.distance_threshold2;
normal_distance_weight2 = config.normal_distance_weight2;
eps_angle = config.eps_angle;
leaf_size = config.leaf_size;
optimize_coefficients = config.optimize_coefficients;
optimize_coefficients2 = config.optimize_coefficients2;
downsample = config.skip_downsample;

}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");


  ros::NodeHandle nh;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("aligned_1", 1, true);
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_2", 1, true);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_3", 1, true);
  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_4", 1, true);


  dynamic_reconfigure::Server<registration_localization::SEGConfig> server;
  dynamic_reconfigure::Server<registration_localization::SEGConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

std_msgs::String filename;    

    std::stringstream ss;
    ss << "/home/mike/marker/" << argv[1];

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZI> );
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (ss.str(), *cloudA) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }

  std::cerr << "PointCloud before filtering: " << cloudA->width * cloudA->height << " data points." << std::endl;
pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>), cloud_p (new pcl::PointCloud<pcl::PointXYZI>), cloud_p2 (new pcl::PointCloud<pcl::PointXYZI>),  cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZI>), cloud_f (new pcl::PointCloud<pcl::PointXYZI>);

sensor_msgs::PointCloud2 output, output1, output2, output3;

ROS_INFO("LEAF %f",leaf_size);
ROS_INFO("k %d",k_search);


pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_save (new pcl::PointCloud<pcl::PointXYZI>);

reco = false;
while(ros::ok()){
  pcl::toROSMsg(*cloudA, output);
output.header.frame_id = "base_link";
output.header.stamp = ros::Time::now();
pub.publish(output);
ros::spinOnce();
pcl::toPCLPointCloud2 ( *cloudA,*cloud_blob);
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (leaf_size,leaf_size, leaf_size);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
if(downsample){
cloud_filtered.swap(cloudA);
  std::cerr << "using " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
}
  pcl::toROSMsg(*cloud_filtered, output1);
output1.header.frame_id = "base_link";
output1.header.stamp = ros::Time::now();
pub1.publish(output1);
ros::spinOnce();
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg; 
    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg2; 
  // Optional
  seg.setOptimizeCoefficients (optimize_coefficients);
  seg2.setOptimizeCoefficients (optimize_coefficients2);




  // Mandatory
  seg.setModelType (11);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (max_iterations);
  seg.setDistanceThreshold (distance_threshold);
  seg.setNormalDistanceWeight(normal_distance_weight);
  seg.setEpsAngle(eps_angle);
  // Mandatory
  seg2.setModelType (11);
  seg2.setMethodType (pcl::SAC_RANSAC);
  seg2.setMaxIterations (max_iterations);
  seg2.setDistanceThreshold (distance_threshold2);
  seg2.setNormalDistanceWeight(normal_distance_weight2);
  seg2.setEpsAngle(eps_angle);
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
 ne.setInputCloud (cloud_filtered);

ne.setSearchSurface (cloudA);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (radius_search);
  ne.setKSearch (k_search);



  // Compute the features
  ne.compute (*cloud_normals);



int ctr = 0;
while(ctr < 30 && reco == false && ros::ok()){
	++ctr;




 // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals(cloud_normals); 
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;

    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);    
    cloud_filtered2.swap (cloud_p);
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);

    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers);
    extract_normals.setNegative (false);
    extract_normals.filter (*cloud_normals_f);
    cloud_normals2.swap (cloud_normals_f);
    extract_normals.setNegative (true);
    extract_normals.filter (*cloud_normals_f);
    cloud_normals.swap (cloud_normals_f);

    seg2.setInputCloud (cloud_filtered2);
    seg2.setInputNormals(cloud_normals2); 
    seg2.segment (*inliers, *coefficients);

    // Extract the inliers
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p2);
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
    std::cerr << "PointCloud representing the second planar component: " << cloud_p2->width * cloud_p2->height << " data points." << std::endl;



  pcl::toROSMsg(*cloud_filtered2, output2);
output2.header.frame_id = "base_link";
output2.header.stamp = ros::Time::now();
  pcl::toROSMsg(*cloud_p2, output3);
output3.header.frame_id = "base_link";
output3.header.stamp = ros::Time::now();

  pcl::toROSMsg(*cloud_filtered, output1);
output1.header.frame_id = "base_link";
output1.header.stamp = ros::Time::now();
if(argc > 2){
//Populate the PCL pointcloud with the ROS message
std::stringstream ss2;
    ss2 << "/home/mike/marker/" << argv[2] << ctr << ".pcd";
std_msgs::String filename;
filename.data = ss2.str();
  pcl::io::savePCDFileASCII (filename.data.c_str(), *cloud_filtered);
ROS_INFO("Saved to %s", filename.data.c_str());
}
pub1.publish(output1);
pub2.publish(output2);
pub3.publish(output3);
ros::spinOnce();








}
reco = false;




}

 
std::cout << "done" << std::endl;
// Publish the data



}

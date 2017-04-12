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
#include <pcl/io/pcd_io.h>
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include <ctime>
#include <boost/make_shared.hpp>


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  ros::Publisher pub;
  ros::Publisher pub1;
  ros::Publisher pub2;
  ros::Publisher pub3;
  ros::Publisher pub4;
  ros::Publisher pub5;
  ros::Publisher pub6;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
 // pcl::PointCloud<pcl::PointXYZI>::Ptr marker (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudE (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudF (new pcl::PointCloud<pcl::PointXYZI> );

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_aligned2 (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_aligned2 (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC_aligned2 (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD_aligned2 (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_aligned (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_aligned (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC_aligned (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD_aligned (new pcl::PointCloud<pcl::PointXYZI> );

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("aligned_1", 1, true);
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_2", 1, true);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_3", 1, true);
  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_4", 1, true);
  pub4 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_5", 1, true);
  pub5 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_6", 1, true);
  pub6 = nh.advertise<sensor_msgs::PointCloud2> ("unaligned", 1, true);

ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker0", 0 , true);
ros::Publisher vis_pub1 = nh.advertise<visualization_msgs::Marker>( "visualization_marker1", 0 , true);
ros::Publisher vis_pub2 = nh.advertise<visualization_msgs::Marker>( "visualization_marker2", 0 , true);
ros::Publisher vis_pub3 = nh.advertise<visualization_msgs::Marker>( "visualization_marker3", 0 , true);

  // load a file
  /*  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/oldmarker.pcd", *marker) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
*/
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/t1.pcd", *cloudA) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/t2.pcd", *cloudB) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/t3.pcd", *cloudC) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/t4.pcd", *cloudD) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }

   sensor_msgs::PointCloud2 output1;
  pcl::toROSMsg(*cloudA, output1);
output1.header.frame_id = "base_link";
output1.header.stamp = ros::Time::now();
   sensor_msgs::PointCloud2 output2;
  pcl::toROSMsg(*cloudB, output2);
output2.header.frame_id = "base_link";
output2.header.stamp = ros::Time::now();
   sensor_msgs::PointCloud2 output3;
  pcl::toROSMsg(*cloudC, output3);
output3.header.frame_id = "base_link";
output3.header.stamp = ros::Time::now();
   sensor_msgs::PointCloud2 output4;
  pcl::toROSMsg(*cloudD, output4);
output4.header.frame_id = "base_link";
output4.header.stamp = ros::Time::now();
   sensor_msgs::PointCloud2 output5;
  pcl::toROSMsg(*cloudA, output5);
output5.header.frame_id = "base_link";
output5.header.stamp = ros::Time::now();

// Publish the data
  pub.publish (output1);
  pub1.publish (output2);
  pub2.publish (output3);
  pub3.publish (output4);
  pub4.publish (output5);
ros::spinOnce();
std::cout << "Done" << std::endl;


}

#include <ros/ros.h>
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

#include <boost/thread/thread.hpp>

#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.01f;
float support_size = 0.001f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
bool setUnseenToMaxRange = true;

  pcl::visualization::PCLVisualizer viewer ("3D Viewer");

  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
std_msgs::String filename;

ros::Publisher pub;
ros::Publisher pub2;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  
ROS_INFO("GOT A CLOUD");
sensor_msgs::PointCloud2 cloud_msg2 = *cloud_msg;
//fix the naming discrepancy between ROS and PCL (from "intensities" to "intensity")
cloud_msg2.fields[3].name = "intensity";
//Create a PCL pointcloud
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>& point_cloud = *cloud_filtered;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
//Populate the PCL pointcloud with the ROS message
pcl::fromROSMsg(cloud_msg2,*cloud);
// Create the filtering object
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("intensity");
  pass.setFilterLimits (1000, 5000);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);


    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);
 // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;   
  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges (far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();
   std::cout << range_image << "\n";
  
  // --------------------------------
  // -----Extract NARF keypoints-----
  // --------------------------------
  pcl::RangeImageBorderExtractor range_image_border_extractor;
  pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage (&range_image);
  narf_keypoint_detector.getParameters ().support_size = support_size;
  //narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
  //narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;
  
  pcl::PointCloud<int> keypoint_indices;
  narf_keypoint_detector.compute (keypoint_indices);
  std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

  // ----------------------------------------------
  // -----Show keypoints in range image widget-----
  // ----------------------------------------------
  //for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    //range_image_widget.markPoint (keypoint_indices.points[i]%range_image.width,
                                  //keypoint_indices.points[i]/range_image.width);
  
  // -------------------------------------
  // -----Show keypoints in 3D viewer-----
  // -------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
  keypoints.points.resize (keypoint_indices.points.size ());
  for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();



  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  viewer.setBackgroundColor (1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
  viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  //viewer.addCoordinateSystem (1.0f, "global");
  //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  viewer.initCameraParameters ();
  //setViewerPose (viewer, range_image.getTransformationToWorldSystem ());
  
  // --------------------------
  // -----Show range image-----
  // --------------------------
  range_image_widget.showRangeImage (range_image);



   sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*keypoints_ptr, output);
output.header.frame_id = "base_link";
  pub.publish (output);

   sensor_msgs::PointCloud2 output2;
  pcl::toROSMsg(*cloud_filtered, output2);
output.header.frame_id = "base_link";
  pub2.publish (output2);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
 

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("assembled_cloud", 1, cloud_cb);
	pub = nh.advertise<sensor_msgs::PointCloud2> ("keypoint_cloud", 1);
	pub2 = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1);
ros::spin();

}

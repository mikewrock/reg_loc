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

#define DEBUG 0

#define L12 0.00703381219
#define DOT 0.439
#define L23 0.50
#define LACC 0.05
#define LACC2 0.15
int cloudctr = 0;



Eigen::Matrix3f marker_loc_world;

	Eigen::Matrix4f t_mat2;

class fitnessClass
{
public:
	fitnessClass() { }
	float score;
	int P1;
	int P2;
	int P3;
};


float dot_product(Eigen::Vector3f first,Eigen::Vector3f second){

	return first(0)*second(0) + first(1)*second(1) + first(2)*second(2);


}
float check_planes(Eigen::Vector4f first,Eigen::Vector4f second){
	Eigen::Vector3f normal1, normal2, normal3, dots;
	normal1(0) = first(0);
	normal1(1) = first(1);
	normal1(2) = first(2);
	normal2(0) = second(0);
	normal2(1) = second(1);
	normal2(2) = second(2);
	normal3(0) = 0;
	normal3(1) = 0;
	normal3(2) = 1;
	dots(0) = dot_product(normal1,normal2);
	dots(1) = dot_product(normal1,normal3);
	dots(2) = dot_product(normal3,normal2);
//ROS_INFO("Plane value %f -- %f -- %f",dots(0), dots(1), dots(2));
	if(fabs((fabs(dots(0))-DOT)) < LACC && (fabs(dots(1)) + fabs(dots(2))) < LACC2) return dots(0);
	else return 200;
	


}
float length_of(Eigen::Vector3f first){

	return sqrt(first(0)*first(0) + first(1)*first(1) + first(2)*first(2) );


}

Eigen::Vector3f vector_of(pcl::PointXYZI first,pcl::PointXYZI second){

	Eigen::Vector3f ret;
	ret(0) = second.x - first.x;
	ret(1) = second.y - first.y;
	ret(2) = second.z - first.z;
	return ret;


}

Eigen::Vector3f cross_product(Eigen::Vector3f first,Eigen::Vector3f second){

	Eigen::Vector3f ret;
	ret(0) = first(1)*second(2)-first(2)*second(1);
	ret(1) = first(2)*second(0)-first(0)*second(2);
	ret(2) = first(0)*second(1)-first(1)*second(0);
	return ret;


}

Eigen::Vector3f cross_product(Eigen::Vector4f first,Eigen::Vector4f second){

	Eigen::Vector3f ret;
	ret(0) = first(1)*second(2)-first(2)*second(1);
	ret(1) = first(2)*second(0)-first(0)*second(2);
	ret(2) = first(0)*second(1)-first(1)*second(0);
	return ret;


}

Eigen::Vector3f normalized_cross_product(Eigen::Vector4f first,Eigen::Vector3f second){

	Eigen::Vector3f ret;
	ret(0) = first(1)*second(2)-first(2)*second(1);
	ret(1) = first(2)*second(0)-first(0)*second(2);
	ret(2) = first(0)*second(1)-first(1)*second(0);
	float m = sqrt(pow(ret(0),2)+pow(ret(1),2)+pow(ret(2),2));
	ret(0) = ret(0)/m;
	ret(1) = ret(1)/m;
	ret(2) = ret(2)/m;
	return ret;


}

Eigen::Vector3f normalized_cross_product(Eigen::Vector3f first,Eigen::Vector4f second){

	Eigen::Vector3f ret;
	ret(0) = first(1)*second(2)-first(2)*second(1);
	ret(1) = first(2)*second(0)-first(0)*second(2);
	ret(2) = first(0)*second(1)-first(1)*second(0);
	float m = sqrt(pow(ret(0),2)+pow(ret(1),2)+pow(ret(2),2));
	ret(0) = ret(0)/m;
	ret(1) = ret(1)/m;
	ret(2) = ret(2)/m;
	return ret;


}
Eigen::Vector3f normalized_cross_product(Eigen::Vector3f first,Eigen::Vector3f second){

	Eigen::Vector3f ret;
	ret(0) = first(1)*second(2)-first(2)*second(1);
	ret(1) = first(2)*second(0)-first(0)*second(2);
	ret(2) = first(0)*second(1)-first(1)*second(0);
	float m = sqrt(pow(ret(0),2)+pow(ret(1),2)+pow(ret(2),2));
	ret(0) = ret(0)/m;
	ret(1) = ret(1)/m;
	ret(2) = ret(2)/m;
	return ret;


}
Eigen::Vector3f normalized_cross_product(Eigen::Vector4f first,Eigen::Vector4f second){

	Eigen::Vector3f ret;
	ret(0) = first(1)*second(2)-first(2)*second(1);
	ret(1) = first(2)*second(0)-first(0)*second(2);
	ret(2) = first(0)*second(1)-first(1)*second(0);
	float m = sqrt(pow(ret(0),2)+pow(ret(1),2)+pow(ret(2),2));
	ret(0) = ret(0)/m;
	ret(1) = ret(1)/m;
	ret(2) = ret(2)/m;
	return ret;


}

Eigen::Matrix4f locate_marker(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src, Eigen::Matrix3f &marker_location){
	

	Eigen::Matrix4f transform_mat;


  std::cerr << "PointCloud before filtering: " << cloud_src->width * cloud_src->height << " data points." << std::endl;

pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>), cloud_p (new pcl::PointCloud<pcl::PointXYZI>), cloud_f (new pcl::PointCloud<pcl::PointXYZI>);

pcl::toPCLPointCloud2 ( *cloud_src,*cloud_blob);
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.02f, 0.02f, 0.02f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg; 
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (11);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.02);
  seg.setNormalDistanceWeight(0.1);
  seg.setEpsAngle(0.05);
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud (cloud_filtered);

ne.setSearchSurface (cloud_src);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.15);

  // Compute the features
  ne.compute (*cloud_normals);
 
std::vector<Eigen::Vector4f> planes;
Eigen::Vector4f plane;
 // While 30% of the original cloud is still there
ROS_INFO("Points %d", nr_points);
//cloud_filtered->points.size () > 0.5 * nr_points &&
int cnts = 0;
i = 0;
int j = 0;
int k = 0;
bool marker_flag = false;
Eigen::Vector3f intersection, holder;
Eigen::Vector3f best_intersection(100,100,100);
float best_value = 100;
float value;
Eigen::Vector3f purpendicular,parallel1, parallel2;





  while ( ros::ok() && cnts < 30 && best_value > 1)
  {
++cnts;
ROS_INFO("Points left %d, counts %d", cloud_filtered->points.size(), cnts);
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
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

	plane(0) = coefficients->values[0];
	plane(1) = coefficients->values[1];
	plane(2) = coefficients->values[2];
	plane(3) = coefficients->values[3];

	planes.push_back(plane);
i = 0;
while( i < (planes.size()-1) && planes.size() >= 2 && best_value > 1){
	j = i+1; 
	while(  j < (planes.size())){
			value = check_planes(planes.at(i),planes.at(j));	
			
			if(fabs(fabs(value)-DOT) < best_value){
				float capr = ( planes.at(i)(2) - (planes.at(i)(0)/planes.at(j)(0))*planes.at(j)(2));
				float daps = (planes.at(i)(3) - (planes.at(i)(0)/planes.at(j)(0))*planes.at(j)(3));
				float bapq = (planes.at(i)(1) - (planes.at(i)(0)/planes.at(j)(0))*planes.at(j)(1));
				float batu = (planes.at(i)(1) - (planes.at(i)(0)/planes.at(k)(0))*planes.at(k)(1));
				intersection(2) = 0; 	
				intersection(1) = (-planes.at(i)(3) + (planes.at(i)(0)/planes.at(j)(0))*planes.at(j)(3))/(planes.at(i)(1)-(planes.at(i)(0)/planes.at(j)(0))*planes.at(j)(1));
				intersection(0)	= (-planes.at(i)(1)*intersection(1)-planes.at(i)(3))/planes.at(i)(0);
				purpendicular(0) = 0;
				purpendicular(1) = 0;
				purpendicular(2) = 1;
				holder = normalized_cross_product(planes.at(i),planes.at(j));
				if(holder(2) > 0){
				if(value > 0) parallel1 = normalized_cross_product(purpendicular,planes.at(i));

				else	parallel1 = normalized_cross_product(planes.at(i),purpendicular);
					parallel2 = normalized_cross_product(purpendicular,planes.at(j));
				}
				else{
				if(value > 0) {parallel2 = normalized_cross_product(purpendicular,planes.at(i));


					parallel1 = normalized_cross_product(purpendicular,planes.at(j));
				}
				else	{parallel1 = normalized_cross_product(planes.at(i),purpendicular);
					parallel2 = normalized_cross_product(purpendicular,planes.at(j));
}
				}
				ROS_INFO("Found a marker at %f - %f - %f\nvalue - %f - holder - %f", intersection(0),intersection(1),intersection(2),value, holder(2)); 
				best_value = fabs(fabs(value)-DOT);
				marker_flag = true;

			}
	++j;
	}
++i;
}

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);

    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers);
    extract_normals.setNegative (true);
    extract_normals.filter (*cloud_normals_f);
    cloud_normals.swap (cloud_normals_f);


}



if(marker_flag == false) ROS_INFO("Couldn't find marker");

	float P1x = intersection(0);
	float P1y = intersection(1);
	float P1z = intersection(2);
	float P2x = intersection(0) + parallel1(0);
	float P2y = intersection(1) + parallel1(1);
	float P2z = intersection(2) + parallel1(2);
	float P3x = intersection(0) + parallel2(0);
	float P3y = intersection(1) + parallel2(1);
	float P3z = intersection(2) + parallel2(2);

	ROS_INFO("Using:");
	ROS_INFO("P1: %f - %f - %f", P1x,P1y,P1z);
	ROS_INFO("P2: %f - %f - %f",P2x,P2y,P2z);
	ROS_INFO("P3: %f - %f - %f", P3x,P3y,P3z);	

	marker_location(0,0) = P1x;
	marker_location(0,1) = P1y;
	marker_location(0,2) = P1z;
	marker_location(1,0) = P2x;
	marker_location(1,1) = P2y;
	marker_location(1,2) = P2z;
	marker_location(2,0) = P3x;
	marker_location(2,1) = P3y;
	marker_location(2,2) = P3z;

	//Calculate rotation matrix
	
	Eigen::Matrix<float, 4, 4> A_mat;
	Eigen::Vector3f a,n,c,o;
	float mn = sqrt(pow(P3x-P2x,2)+pow(P3y-P2y,2)+pow(P3z-P2z,2));
	float mc = sqrt(pow(P1x-P2x,2)+pow(P1y-P2y,2)+pow(P1z-P2z,2));
	n(0) = (P3x-P2x)/mn;
	n(1) = (P3y-P2y)/mn;
	n(2) = (P3z-P2z)/mn;
	c(0) = (P1x-P2x)/mc;
	c(1) = (P1y-P2y)/mc;
	c(2) = (P1z-P2z)/mc;
	a = normalized_cross_product(n,c);
	o = cross_product(a,n);
	A_mat(0,0) = n(0);
	A_mat(0,1) = o(0);
	A_mat(0,2) = a(0);
	A_mat(0,3) = (P1x+P2x+P3x)/3;
	A_mat(1,0) = n(1);
	A_mat(1,1) = o(1);
	A_mat(1,2) = a(1);
	A_mat(1,3) = (P1y+P2y+P3y)/3;
	A_mat(2,0) = n(2);
	A_mat(2,1) = o(2);
	A_mat(2,2) = a(2);
	A_mat(2,3) = (P1z+P2z+P3z)/3;
	A_mat(3,0) = 0;
	A_mat(3,1) = 0;
	A_mat(3,2) = 0;
	A_mat(3,3) = 1;
	
	ROS_INFO("Done \n\n\n");

	return A_mat;

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

    std::stringstream ss;
    ss << "/home/mike/marker/" << argv[1] << ".pcd";

  ros::Publisher pub;
  ros::Publisher pub1;
  ros::Publisher pub2;
  ros::Publisher pub3;
  ros::Publisher pub4;
  ros::Publisher pub5;
  ros::Publisher pub6;

 // pcl::PointCloud<pcl::PointXYZI>::Ptr marker (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD (new pcl::PointCloud<pcl::PointXYZI> );

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_aligned2 (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_aligned2 (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC_aligned2 (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD_aligned2 (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_aligned (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_aligned (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC_aligned (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD_aligned (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudfinal_aligned (new pcl::PointCloud<pcl::PointXYZI> );

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



//setVerbosityLevel(pcl::console::L_VERBOSE); 
 if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/plane1.pcd", *cloudA) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/plane2.pcd", *cloudB) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/plane3.pcd", *cloudC) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/plane4.pcd", *cloudD) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }Eigen::Matrix3f marker_locA;
Eigen::Matrix3f marker_locB;
Eigen::Matrix3f marker_locC;
Eigen::Matrix3f marker_locD;
Eigen::Matrix4f transformA;
Eigen::Matrix4f transformB;
Eigen::Matrix4f transformC;
Eigen::Matrix4f transformD;
ROS_INFO("Locating Markers");   
sensor_msgs::PointCloud2 output5;
  pcl::toROSMsg(*cloudA, output5);
output5.header.frame_id = "base_link";
output5.header.stamp = ros::Time::now();

  pub4.publish (output5);
ros::spinOnce();
transformA = locate_marker(cloudA, marker_locA);
visualization_msgs::Marker marker1;
visualization_msgs::Marker marker2;
visualization_msgs::Marker marker3;
visualization_msgs::Marker marker4;
float m_size = 0.01;
marker1.header.frame_id = "base_link";
marker1.header.stamp = ros::Time();
marker1.ns = "my_namespace";
marker1.id = 0;
marker1.type = visualization_msgs::Marker::SPHERE;
marker1.action = visualization_msgs::Marker::ADD;
marker1.pose.position.x = marker_locA(0,0);
marker1.pose.position.y = marker_locA(0,1);
marker1.pose.position.z = marker_locA(0,2);
marker1.pose.orientation.x = 0.0;
marker1.pose.orientation.y = 0.0;
marker1.pose.orientation.z = 0.0;
marker1.pose.orientation.w = 1.0;
marker1.scale.x = 0.1;
marker1.scale.y = 0.1;
marker1.scale.z = 0.1;
marker1.color.a = 1.0; // Don't forget to set the alpha!
marker1.color.r = 1.0;
marker1.color.g = 1.0;
marker1.color.b = 1.0;
vis_pub.publish( marker1 );
marker2.header.frame_id = "base_link";
marker2.header.stamp = ros::Time();
marker2.ns = "my_namespace";
marker2.id = 1;
marker2.type = visualization_msgs::Marker::ARROW;
marker2.action = visualization_msgs::Marker::ADD;

 marker2.points.resize(2);
     marker2.points[0].x = marker_locA(0,0);
     marker2.points[0].y = marker_locA(0,1);
     marker2.points[0].z = marker_locA(0,2);
     marker2.points[1].x = marker_locA(1,0);
     marker2.points[1].y = marker_locA(1,1);
     marker2.points[1].z = marker_locA(1,2);
marker2.scale.x = m_size;
marker2.scale.y = m_size;
marker2.scale.z = m_size;
marker2.color.a = 1.0; // Don't forget to set the alpha!
marker2.color.r = 1.0;
marker2.color.g = 0.0;
marker2.color.b = 0.0;
vis_pub1.publish( marker2 );
marker3.header.frame_id = "base_link";
marker3.header.stamp = ros::Time();
marker3.ns = "my_namespace";
marker3.id = 2;
marker3.type = visualization_msgs::Marker::ARROW;
marker3.action = visualization_msgs::Marker::ADD;

 marker3.points.resize(2);
     marker3.points[0].x = marker_locA(0,0);
     marker3.points[0].y = marker_locA(0,1);
     marker3.points[0].z = marker_locA(0,2);
     marker3.points[1].x = marker_locA(2,0);
     marker3.points[1].y = marker_locA(2,1);
     marker3.points[1].z = marker_locA(2,2);
marker3.scale.x = m_size;
marker3.scale.y = m_size;
marker3.scale.z = m_size;
marker3.color.a = 1.0; // Don't forget to set the alpha!
marker3.color.r = 0.0;
marker3.color.g = 0.0;
marker3.color.b = 1.0;
vis_pub2.publish( marker3 );
Eigen::Matrix4f transformAA = transformA.inverse();
pcl::transformPointCloud (*cloudA, *cloudA_aligned, transformAA);
   sensor_msgs::PointCloud2 output1;
  pcl::toROSMsg(*cloudA_aligned, output1);
output1.header.frame_id = "base_link";
output1.header.stamp = ros::Time::now();
  pub.publish (output1);  
ros::spinOnce();
/*transformB = locate_marker(cloudB, marker_locB);
marker1.header.frame_id = "base_link";
marker1.header.stamp = ros::Time();
marker1.ns = "my_namespace";
marker1.id = 0;
marker1.type = visualization_msgs::Marker::SPHERE;
marker1.action = visualization_msgs::Marker::ADD;
marker1.pose.position.x = marker_locB(0,0);
marker1.pose.position.y = marker_locB(0,1);
marker1.pose.position.z = marker_locB(0,2);
marker1.pose.orientation.x = 0.0;
marker1.pose.orientation.y = 0.0;
marker1.pose.orientation.z = 0.0;
marker1.pose.orientation.w = 1.0;
marker1.scale.x = 0.1;
marker1.scale.y = 0.1;
marker1.scale.z = 0.1;
marker1.color.a = 1.0; // Don't forget to set the alpha!
marker1.color.r = 1.0;
marker1.color.g = 1.0;
marker1.color.b = 1.0;
vis_pub.publish( marker1 );
marker2.header.frame_id = "base_link";
marker2.header.stamp = ros::Time();
marker2.ns = "my_namespace";
marker2.id = 1;
marker2.type = visualization_msgs::Marker::ARROW;
marker2.action = visualization_msgs::Marker::ADD;

 marker2.points.resize(2);
     marker2.points[0].x = marker_locB(0,0);
     marker2.points[0].y = marker_locB(0,1);
     marker2.points[0].z = marker_locB(0,2);
     marker2.points[1].x = marker_locB(1,0);
     marker2.points[1].y = marker_locB(1,1);
     marker2.points[1].z = marker_locB(1,2);
marker2.scale.x = m_size;
marker2.scale.y = m_size;
marker2.scale.z = m_size;
marker2.color.a = 1.0; // Don't forget to set the alpha!
marker2.color.r = 1.0;
marker2.color.g = 0.0;
marker2.color.b = 0.0;
vis_pub1.publish( marker2 );
marker3.header.frame_id = "base_link";
marker3.header.stamp = ros::Time();
marker3.ns = "my_namespace";
marker3.id = 2;
marker3.type = visualization_msgs::Marker::ARROW;
marker3.action = visualization_msgs::Marker::ADD;

 marker3.points.resize(2);
     marker3.points[0].x = marker_locB(0,0);
     marker3.points[0].y = marker_locB(0,1);
     marker3.points[0].z = marker_locB(0,2);
     marker3.points[1].x = marker_locB(2,0);
     marker3.points[1].y = marker_locB(2,1);
     marker3.points[1].z = marker_locB(2,2);
marker3.scale.x = m_size;
marker3.scale.y = m_size;
marker3.scale.z = m_size;
marker3.color.a = 1.0; // Don't forget to set the alpha!
marker3.color.r = 0.0;
marker3.color.g = 0.0;
marker3.color.b = 1.0;
vis_pub2.publish( marker3 );
Eigen::Matrix4f transformAB = transformA*transformB.inverse();
pcl::transformPointCloud (*cloudB, *cloudB_aligned2, transformAB);
pcl::transformPointCloud (*cloudB_aligned2, *cloudB_aligned, transformAA);
   sensor_msgs::PointCloud2 output2;
  pcl::toROSMsg(*cloudB_aligned, output2);
output2.header.frame_id = "base_link";
output2.header.stamp = ros::Time::now();
  pub1.publish (output2);
  pcl::toROSMsg(*cloudC, output5);
output5.header.frame_id = "base_link";
output5.header.stamp = ros::Time::now();

  pub4.publish (output5);
ros::spinOnce();
*/transformC = locate_marker(cloudC, marker_locC);

  pcl::toROSMsg(*cloudC, output5);
output5.header.frame_id = "base_link";
output5.header.stamp = ros::Time::now();

  pub4.publish (output5);
marker1.header.frame_id = "base_link";
marker1.header.stamp = ros::Time();
marker1.ns = "my_namespace";
marker1.id = 0;
marker1.type = visualization_msgs::Marker::SPHERE;
marker1.action = visualization_msgs::Marker::ADD;
marker1.pose.position.x = marker_locC(0,0);
marker1.pose.position.y = marker_locC(0,1);
marker1.pose.position.z = marker_locC(0,2);
marker1.pose.orientation.x = 0.0;
marker1.pose.orientation.y = 0.0;
marker1.pose.orientation.z = 0.0;
marker1.pose.orientation.w = 1.0;
marker1.scale.x = 0.1;
marker1.scale.y = 0.1;
marker1.scale.z = 0.1;
marker1.color.a = 1.0; // Don't forget to set the alpha!
marker1.color.r = 1.0;
marker1.color.g = 1.0;
marker1.color.b = 1.0;
vis_pub.publish( marker1 );
marker2.header.frame_id = "base_link";
marker2.header.stamp = ros::Time();
marker2.ns = "my_namespace";
marker2.id = 1;
marker2.type = visualization_msgs::Marker::ARROW;
marker2.action = visualization_msgs::Marker::ADD;

 marker2.points.resize(2);
     marker2.points[0].x = marker_locC(0,0);
     marker2.points[0].y = marker_locC(0,1);
     marker2.points[0].z = marker_locC(0,2);
     marker2.points[1].x = marker_locC(1,0);
     marker2.points[1].y = marker_locC(1,1);
     marker2.points[1].z = marker_locC(1,2);
marker2.scale.x = m_size;
marker2.scale.y = m_size;
marker2.scale.z = m_size;
marker2.color.a = 1.0; // Don't forget to set the alpha!
marker2.color.r = 1.0;
marker2.color.g = 0.0;
marker2.color.b = 0.0;
vis_pub1.publish( marker2 );
marker3.header.frame_id = "base_link";
marker3.header.stamp = ros::Time();
marker3.ns = "my_namespace";
marker3.id = 2;
marker3.type = visualization_msgs::Marker::ARROW;
marker3.action = visualization_msgs::Marker::ADD;

 marker3.points.resize(2);
     marker3.points[0].x = marker_locC(0,0);
     marker3.points[0].y = marker_locC(0,1);
     marker3.points[0].z = marker_locC(0,2);
     marker3.points[1].x = marker_locC(2,0);
     marker3.points[1].y = marker_locC(2,1);
     marker3.points[1].z = marker_locC(2,2);
marker3.scale.x = m_size;
marker3.scale.y = m_size;
marker3.scale.z = m_size;
marker3.color.a = 1.0; // Don't forget to set the alpha!
marker3.color.r = 0.0;
marker3.color.g = 0.0;
marker3.color.b = 1.0;
vis_pub2.publish( marker3 );
Eigen::Matrix4f transformAC = transformA*transformC.inverse();
pcl::transformPointCloud (*cloudC, *cloudC_aligned2, transformAC);
pcl::transformPointCloud (*cloudC_aligned2, *cloudC_aligned, transformAA);
   sensor_msgs::PointCloud2 output3;
*cloudfinal_aligned = *cloudA_aligned+*cloudC_aligned;  
  pcl::toROSMsg(*cloudfinal_aligned, output3);
output3.header.frame_id = "base_link";
output3.header.stamp = ros::Time::now();
  pub2.publish (output3);
ros::spinOnce();
transformD = locate_marker(cloudD, marker_locD);
marker1.header.frame_id = "base_link";
marker1.header.stamp = ros::Time();
marker1.ns = "my_namespace";
marker1.id = 0;
marker1.type = visualization_msgs::Marker::SPHERE;
marker1.action = visualization_msgs::Marker::ADD;
marker1.pose.position.x = marker_locD(0,0);
marker1.pose.position.y = marker_locD(0,1);
marker1.pose.position.z = marker_locD(0,2);
marker1.pose.orientation.x = 0.0;
marker1.pose.orientation.y = 0.0;
marker1.pose.orientation.z = 0.0;
marker1.pose.orientation.w = 1.0;
marker1.scale.x = 0.1;
marker1.scale.y = 0.1;
marker1.scale.z = 0.1;
marker1.color.a = 1.0; // Don't forget to set the alpha!
marker1.color.r = 1.0;
marker1.color.g = 1.0;
marker1.color.b = 1.0;
vis_pub.publish( marker1 );
marker2.header.frame_id = "base_link";
marker2.header.stamp = ros::Time();
marker2.ns = "my_namespace";
marker2.id = 1;
marker2.type = visualization_msgs::Marker::ARROW;
marker2.action = visualization_msgs::Marker::ADD;

 marker2.points.resize(2);
     marker2.points[0].x = marker_locD(0,0);
     marker2.points[0].y = marker_locD(0,1);
     marker2.points[0].z = marker_locD(0,2);
     marker2.points[1].x = marker_locD(1,0);
     marker2.points[1].y = marker_locD(1,1);
     marker2.points[1].z = marker_locD(1,2);
marker2.scale.x = m_size;
marker2.scale.y = m_size;
marker2.scale.z = m_size;
marker2.color.a = 1.0; // Don't forget to set the alpha!
marker2.color.r = 1.0;
marker2.color.g = 0.0;
marker2.color.b = 0.0;
vis_pub1.publish( marker2 );
marker3.header.frame_id = "base_link";
marker3.header.stamp = ros::Time();
marker3.ns = "my_namespace";
marker3.id = 2;
marker3.type = visualization_msgs::Marker::ARROW;
marker3.action = visualization_msgs::Marker::ADD;

 marker3.points.resize(2);
     marker3.points[0].x = marker_locD(0,0);
     marker3.points[0].y = marker_locD(0,1);
     marker3.points[0].z = marker_locD(0,2);
     marker3.points[1].x = marker_locD(2,0);
     marker3.points[1].y = marker_locD(2,1);
     marker3.points[1].z = marker_locD(2,2);
marker3.scale.x = m_size;
marker3.scale.y = m_size;
marker3.scale.z = m_size;
marker3.color.a = 1.0; // Don't forget to set the alpha!
marker3.color.r = 0.0;
marker3.color.g = 0.0;
marker3.color.b = 1.0;
vis_pub2.publish( marker3 );
   sensor_msgs::PointCloud2 output4;
Eigen::Matrix4f transformAD = transformA*transformD.inverse();
pcl::transformPointCloud (*cloudD, *cloudD_aligned2, transformAD);
pcl::transformPointCloud (*cloudD_aligned2, *cloudD_aligned, transformAA);

  pcl::toROSMsg(*cloudD, output5);
output5.header.frame_id = "base_link";
output5.header.stamp = ros::Time::now();

  pub4.publish (output5);
*cloudfinal_aligned = *cloudfinal_aligned + *cloudD_aligned;
  pcl::toROSMsg(*cloudfinal_aligned, output4);
output4.header.frame_id = "base_link";
output4.header.stamp = ros::Time::now();

  pub3.publish (output4);
ros::spinOnce();
////////////////////////////////////////


  pcl::toROSMsg(*cloudA_aligned, output1);
output1.header.frame_id = "base_link";
output1.header.stamp = ros::Time::now();
sensor_msgs::PointCloud2 output2;
  pcl::toROSMsg(*cloudB_aligned, output2);
output2.header.frame_id = "base_link";
output2.header.stamp = ros::Time::now();
  pcl::toROSMsg(*cloudD_aligned, output4);
output4.header.frame_id = "base_link";
output4.header.stamp = ros::Time::now();
  
  pub1.publish (output1);
  pub2.publish (output2);
  pub3.publish (output4);


ros::spinOnce();
std::cout << "Done" << std::endl;


}

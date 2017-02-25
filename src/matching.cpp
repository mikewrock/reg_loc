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
#define DEBUG 0
#define L1 0.2513803274
#define L2 0.3254835709
#define L3 0.196127621
#define LAcc 0.01
int cloudctr = 0;
Eigen::Matrix3f marker_loc_world;

	Eigen::Matrix4f t_mat2;

float dot_product(Eigen::Vector3f first,Eigen::Vector3f second){

	return first(0)*second(0) + first(1)*second(1) + first(2)*second(2);


}

Eigen::Matrix4f locate_marker(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src, Eigen::Matrix3f &marker_location){
	
	std::vector<int> P1;
	std::vector<int> P2;
	std::vector<int> P3;
	float P1x = 0;
	float P1y = 0;
	float P1z = 0;
	float P2x = 0;
	float P2y = 0;
	float P2z = 0;
	float P3x = 0;
	float P3y = 0;
	float P3z = 0;
	int cnt;
	Eigen::Matrix4f transform_mat;
	// Filter for intensity
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI> );
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud (cloud_src);
	pass.setFilterFieldName ("intensity");
	pass.setFilterLimits (1050, 1500);
	pass.filter (*cloud_filtered);

	if(cloud_filtered->points.size() < 3){

		ROS_INFO("Not enough points in data");
		return transform_mat;

	}
	ROS_INFO("Calculating %d Points",cloud_filtered->points.size());

	//for (size_t q = 0; q < cloud_filtered->points.size (); ++q) ROS_INFO("Point %d: %f - %f - %f",q,cloud_filtered->points[q].x,cloud_filtered->points[q].y,cloud_filtered->points[q].z);
	// Outlier Removal
	for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
		for (size_t j = i+1; j < cloud_filtered->points.size (); ++j){	
			float distance = sqrt(pow(cloud_filtered->points[j].x - cloud_filtered->points[i].x,2) + 
						pow(cloud_filtered->points[j].y - cloud_filtered->points[i].y,2) + 
						  pow(cloud_filtered->points[j].z - cloud_filtered->points[i].z,2)); 	
			for (size_t k = j+1; k < cloud_filtered->points.size (); ++k){	
				float distance2 = sqrt(pow(cloud_filtered->points[k].x - cloud_filtered->points[i].x,2) + 
					pow(cloud_filtered->points[k].y - cloud_filtered->points[i].y,2) + 
					pow(cloud_filtered->points[k].z - cloud_filtered->points[i].z,2));				
				if(fabs(distance - L1) < LAcc && fabs(distance2 - L2) < LAcc){
					P1.push_back(i);
					P2.push_back(j);
					P3.push_back(k);
		
				//ROS_INFO("at j = %d k = %d D1: %f D2: %f",j,k,distance -L1,distance2-L2);	
					}
				if(fabs(distance - L2) < LAcc && fabs(distance2 - L1) < LAcc){
					P1.push_back(i);
					P2.push_back(k);
					P3.push_back(j);

				//ROS_INFO("at j = %d k = %d D1: %f D2: %f",j,k,distance-L2,distance2-L1);
					}
				if(fabs(distance - L1) < LAcc && fabs(distance2 - L3) < LAcc){
					P1.push_back(j);
					P2.push_back(i);
					P3.push_back(k);

				//ROS_INFO("at i = %d k = %d D1: %f D2: %f",i,k,distance-L1,distance2-L3);
					}
				if(fabs(distance - L3) < LAcc && fabs(distance2 - L1) < LAcc){
					P1.push_back(k);
					P2.push_back(i);
					P3.push_back(j);
				//ROS_INFO("at i = %d j = %d D1: %f D2: %f",i,j,distance-L3,distance2-L1);
					}
				if(fabs(distance - L2) < LAcc && fabs(distance2 - L3) < LAcc){
					P1.push_back(j);
					P2.push_back(k);
					P3.push_back(i);
				//ROS_INFO("at i = %d k = %d D1: %f D2: %f",i,k,distance-L2,distance2-L3);
					}
				if(fabs(distance - L3) < LAcc && fabs(distance2 - L2) < LAcc){
					P1.push_back(k);
					P2.push_back(j);
					P3.push_back(i);
				//ROS_INFO("at i = %d j = %d D1: %f D2: %f",i,j,distance-L3,distance2-L2);
					}
				}
			}
		}

	//Remove duplicates
	sort( P1.begin(), P1.end());
	P1.erase( unique( P1.begin(), P1.end() ), P1.end() );
	sort( P2.begin(), P2.end());
	P2.erase( unique( P2.begin(), P2.end() ), P2.end() );
	sort( P3.begin(), P3.end());
	P3.erase( unique( P3.begin(), P3.end() ), P3.end() );
	//for(cnt = 0; cnt < P1.size();++cnt)		ROS_INFO("P1Point %d: at %d: %f - %f - %f", cnt, P1.at(cnt), cloud_filtered->points[ P1.at(cnt)].x, cloud_filtered->points[ P1.at(cnt)].y, cloud_filtered->points[ P1.at(cnt)].z);

	//for(cnt = 0; cnt < P2.size();++cnt)		ROS_INFO("P2Point %d: at %d: %f - %f - %f", cnt, P2.at(cnt), cloud_filtered->points[ P2.at(cnt)].x, cloud_filtered->points[ P2.at(cnt)].y, cloud_filtered->points[ P2.at(cnt)].z);
	
	//for(cnt = 0; cnt < P3.size();++cnt)		ROS_INFO("P3Point %d: at %d: %f - %f - %f", cnt, P3.at(cnt), cloud_filtered->points[ P3.at(cnt)].x, cloud_filtered->points[ P3.at(cnt)].y, cloud_filtered->points[ P3.at(cnt)].z);
	
	//Average clusters together	
	for(int ctr = 0; ctr < P1.size(); ++ctr){
		P1x += cloud_filtered->points[P1.at(ctr)].x;
		P1y += cloud_filtered->points[P1.at(ctr)].y;
		P1z += cloud_filtered->points[P1.at(ctr)].z;
	}
	P1x = P1x/P1.size();
	P1y = P1y/P1.size();
	P1z = P1z/P1.size();
	for(int ctr = 0; ctr < P2.size(); ++ctr){
		P2x += cloud_filtered->points[P2.at(ctr)].x;
		P2y += cloud_filtered->points[P2.at(ctr)].y;
		P2z += cloud_filtered->points[P2.at(ctr)].z;
	}
	P2x = P2x/P2.size();
	P2y = P2y/P2.size();
	P2z = P2z/P2.size();
	for(int ctr = 0; ctr < P3.size(); ++ctr){
		P3x += cloud_filtered->points[P3.at(ctr)].x;
		P3y += cloud_filtered->points[P3.at(ctr)].y;
		P3z += cloud_filtered->points[P3.at(ctr)].z;
	}
	P3x = P3x/P3.size();
	P3y = P3y/P3.size();
	P3z = P3z/P3.size();
		
	ROS_INFO("Location of marker:");
	ROS_INFO("P1: %f - %f - %f", P1x,P1y,P1z);
	ROS_INFO("P2: %f - %f - %f", P2x,P2y,P2z);
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
	/*
	Eigen::Matrix<float, 3, 4> A_mat;
	Eigen::Matrix<float, 3, 4> B_mat;

	A_mat(0,0) = P1x;
	A_mat(0,1) = P1y;
	A_mat(0,2) = P1z;
	A_mat(0,3) = 1;
	A_mat(1,0) = P2x;
	A_mat(1,1) = P2y;
	A_mat(1,2) = P2z;
	A_mat(1,3) = 1;
	A_mat(2,0) = P3x;
	A_mat(2,1) = P3y;
	A_mat(2,2) = P3z;
	A_mat(2,3) = 1;
	B_mat(0,0) = 0;
	B_mat(0,1) = L1;
	B_mat(0,2) = 0;
	B_mat(0,3) = 1;
	B_mat(1,0) = 0;
	B_mat(1,1) = 0;
	B_mat(1,2) = 0;
	B_mat(1,3) = 1;
	B_mat(2,0) = L3;
	B_mat(2,1) = 0;
	B_mat(2,2) = 0;
	B_mat(2,3) = 1;

	Eigen::Matrix4f t_mat = A_mat.colPivHouseholderQr().solve(B_mat);
*/
	Eigen::Vector3f uhat;
	Eigen::Vector3f vhat(1,0,0);
	Eigen::Vector3f xahat;
	Eigen::Vector3f yahat;
	Eigen::Vector3f zahat;
	Eigen::Vector3f xbhat(1,0,0);
	Eigen::Vector3f ybhat(0,1,0);
	Eigen::Vector3f zbhat(0,0,1);

	xahat(0) = (P3x-P2x)/L3;
	xahat(1) = (P3y-P2y)/L3;
	xahat(2) = (P3z-P2z)/L3;
	yahat(0) = (P1x-P2x)/L1;
	yahat(1) = (P1y-P2y)/L1;
	yahat(2) = (P1z-P2z)/L1;
	zahat(0) = xahat(1)*yahat(2)-xahat(2)*yahat(1);
	zahat(1) = xahat(2)*yahat(0)-xahat(0)*yahat(2);
	zahat(2) = xahat(0)*yahat(1)-xahat(1)*yahat(0);
	float lz = sqrt(pow(zahat(0),2)+pow(zahat(1),2)+pow(zahat(2),2));
	zahat(0) = zahat(0)/lz;
	zahat(1) = zahat(1)/lz;
	zahat(2) = zahat(2)/lz;



	//float alpha = acos(dot_product(xbhat,xahat));
	float theta = acos(dot_product(vhat,xahat));

	//float gamma = acos(dot_product(xbhat,xahat));



/*
	rot_mat(0,0) = dot_product(xbhat,xahat);
	rot_mat(0,1) = dot_product(ybhat,xahat);
	rot_mat(0,2) = dot_product(zbhat,xahat);
	rot_mat(1,0) = dot_product(xbhat,yahat);
	rot_mat(1,1) = dot_product(ybhat,yahat);
	rot_mat(1,2) = dot_product(zbhat,yahat);
	rot_mat(2,0) = dot_product(xbhat,zahat);
	rot_mat(2,1) = dot_product(ybhat,zahat);
	rot_mat(2,2) = dot_product(zbhat,zahat);

	for(int i = 0; i < 3; ++i){
		for(int j=0; j<3; ++j){
			transform_mat(i,j) = rot_mat(i,j);
		}
	}
	transform_mat(3,0) = 0;
	transform_mat(3,1) = 0;
	transform_mat(3,2) = 0;
	transform_mat(3,3) = 1;
	transform_mat(0,3) = -1*P2x;
	transform_mat(1,3) = -1*P2y;
	transform_mat(2,3) = -1*P2z;

*/
	Eigen::Matrix4f t_mat;
	t_mat(0,0) = cos(-theta);
	t_mat(0,1) = -sin(-theta);
	t_mat(0,2) = 0;
	t_mat(0,3) = 0;
	t_mat(1,0) = sin(-theta);
	t_mat(1,1) = cos(-theta);
	t_mat(1,2) = 0;
	t_mat(1,3) = 0;
	t_mat(2,0) = 0;
	t_mat(2,1) = 0;
	t_mat(2,2) = 1;
	t_mat(2,3) = 0;
	t_mat(3,0) = 0;
	t_mat(3,1) = 0;
	t_mat(3,2) = 0;
	t_mat(3,3) = 1;

	t_mat2(0,0) = 1;
	t_mat2(0,1) = 0;
	t_mat2(0,2) = 0;
	t_mat2(0,3) = -P2x;
	t_mat2(1,0) = 0;
	t_mat2(1,1) = 1;
	t_mat2(1,2) = 0;
	t_mat2(1,3) = -P2y;
	t_mat2(2,0) = 0;
	t_mat2(2,1) = 0;
	t_mat2(2,2) = 1;
	t_mat2(2,3) = -P2z;
	t_mat2(3,0) = 0;
	t_mat2(3,1) = 0;
	t_mat2(3,2) = 0;
	t_mat2(3,3) = 1;

	return t_mat;	

}

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

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_aligned2 (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_aligned2 (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC_aligned2 (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD_aligned2 (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_aligned (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_aligned (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC_aligned (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD_aligned (new pcl::PointCloud<pcl::PointXYZI> );

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("aligned_1", 1);
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_2", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_3", 1);
  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_4", 1);
  pub4 = nh.advertise<sensor_msgs::PointCloud2> ("unaligned", 1);

ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

	marker_loc_world(0,0) = 0;
	marker_loc_world(0,1) = 0;
	marker_loc_world(0,2) = 0;
	marker_loc_world(1,0) = 0;
	marker_loc_world(1,1) = L1;
	marker_loc_world(1,2) = 0;
	marker_loc_world(2,0) = L3;
	marker_loc_world(2,1) = 0;
	marker_loc_world(2,2) = 0;

setVerbosityLevel(pcl::console::L_VERBOSE); 

  // load a file
  /*  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/oldmarker.pcd", *marker) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }*/
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/mx1.pcd", *cloudA) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/mx2.pcd", *cloudB) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/mx3.pcd", *cloudC) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/mx4.pcd", *cloudD) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
Eigen::Matrix3f marker_locA;
Eigen::Matrix3f marker_locB;
Eigen::Matrix3f marker_locC;
Eigen::Matrix3f marker_locD;
Eigen::Matrix4f transformA;
Eigen::Matrix4f transformB;
Eigen::Matrix4f transformC;
Eigen::Matrix4f transformD;
ROS_INFO("Locating MarkerA");
transformA = locate_marker(cloudA, marker_locA);
pcl::transformPointCloud (*cloudA, *cloudA_aligned, t_mat2);
pcl::transformPointCloud (*cloudA_aligned, *cloudA_aligned2, transformA);
ROS_INFO("Locating MarkerB");
transformB = locate_marker(cloudB, marker_locB);
pcl::transformPointCloud (*cloudB, *cloudB_aligned, t_mat2);
pcl::transformPointCloud (*cloudB_aligned, *cloudB_aligned2, transformB);
ROS_INFO("Locating MarkerC");
transformC = locate_marker(cloudC, marker_locC);
pcl::transformPointCloud (*cloudC, *cloudC_aligned, t_mat2);
pcl::transformPointCloud (*cloudC_aligned, *cloudC_aligned2, transformC);
ROS_INFO("Locating MarkerD");
transformD = locate_marker(cloudD, marker_locD);
pcl::transformPointCloud (*cloudD, *cloudD_aligned, t_mat2);
pcl::transformPointCloud (*cloudD_aligned, *cloudD_aligned2, transformD);
/*

pcl::transformPointCloud (*cloudA, *cloudA_aligned, transformA);
pcl::transformPointCloud (*cloudB, *cloudB_aligned, transformB);
pcl::transformPointCloud (*cloudC, *cloudC_aligned, transformC);
pcl::transformPointCloud (*cloudD, *cloudD_aligned, transformD);
*/
visualization_msgs::Marker marker;
marker.header.frame_id = "base_link";
marker.header.stamp = ros::Time();
marker.ns = "my_namespace";
marker.id = 0;
marker.type = visualization_msgs::Marker::SPHERE_LIST;
marker.action = visualization_msgs::Marker::ADD;

 marker.points.resize(12);
     marker.points[0].x = marker_locA(0,0);
     marker.points[0].y = marker_locA(0,1);
     marker.points[0].z = marker_locA(0,2);
     marker.points[1].x = marker_locA(1,0);
     marker.points[1].y = marker_locA(1,1);
     marker.points[1].z = marker_locA(1,2);
     marker.points[2].x = marker_locA(2,0);
     marker.points[2].y = marker_locA(2,1);
     marker.points[2].z = marker_locA(2,2);
     marker.points[3].x = marker_locB(0,0);
     marker.points[3].y = marker_locB(0,1);
     marker.points[3].z = marker_locB(0,2);
     marker.points[4].x = marker_locB(1,0);
     marker.points[4].y = marker_locB(1,1);
     marker.points[4].z = marker_locB(1,2);
     marker.points[5].x = marker_locB(2,0);
     marker.points[5].y = marker_locB(2,1);
     marker.points[5].z = marker_locB(2,2);
     marker.points[6].x = marker_locC(0,0);
     marker.points[6].y = marker_locC(0,1);
     marker.points[6].z = marker_locC(0,2);
     marker.points[7].x = marker_locC(1,0);
     marker.points[7].y = marker_locC(1,1);
     marker.points[7].z = marker_locC(1,2);
     marker.points[8].x = marker_locC(2,0);
     marker.points[8].y = marker_locC(2,1);
     marker.points[8].z = marker_locC(2,2);
     marker.points[9].x = marker_locD(0,0);
     marker.points[9].y = marker_locD(0,1);
     marker.points[9].z = marker_locD(0,2);
     marker.points[10].x = marker_locD(1,0);
     marker.points[10].y = marker_locD(1,1);
     marker.points[10].z = marker_locD(1,2);
     marker.points[11].x = marker_locD(2,0);
     marker.points[11].y = marker_locD(2,1);
     marker.points[11].z = marker_locD(2,2);
marker.scale.x = 0.1;
marker.scale.y = 0.1;
marker.scale.z = 0.1;
marker.color.a = 1.0; // Don't forget to set the alpha!
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
vis_pub.publish( marker );

////////////////////////////////////////


   sensor_msgs::PointCloud2 output1;
  pcl::toROSMsg(*cloudA_aligned2, output1);
output1.header.frame_id = "base_link";
   sensor_msgs::PointCloud2 output2;
  pcl::toROSMsg(*cloudB_aligned2, output2);
output2.header.frame_id = "base_link";
   sensor_msgs::PointCloud2 output3;
  pcl::toROSMsg(*cloudC_aligned2, output3);
output3.header.frame_id = "base_link";
   sensor_msgs::PointCloud2 output4;
  pcl::toROSMsg(*cloudD_aligned2, output4);
output4.header.frame_id = "base_link";
  

/*
   sensor_msgs::PointCloud2 output1;
  pcl::toROSMsg(*temp1, output1);
output1.header.frame_id = "base_link";
   sensor_msgs::PointCloud2 output2;
  pcl::toROSMsg(*temp2, output2);
output2.header.frame_id = "base_link";
   sensor_msgs::PointCloud2 output3;
  pcl::toROSMsg(*temp3, output3);
output3.header.frame_id = "base_link";
   sensor_msgs::PointCloud2 output4;
  pcl::toROSMsg(*temp4, output4);
output4.header.frame_id = "base_link";


    sensor_msgs::PointCloud2 output5;
 pcl::toROSMsg(*target4, output5);
output5.header.frame_id = "base_link";
*/
// Publish the data
  pub.publish (output1);
  pub1.publish (output2);
  pub2.publish (output3);
  pub3.publish (output4);
ros::spinOnce();
std::cout << "Done" << std::endl;


}

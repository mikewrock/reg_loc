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

#define L12 0.30
#define L13 0.40
#define L23 0.50
#define LACC 0.005
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
	float angle;
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
float LAcc = LACC;
while(LAcc < 0.1){
	for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
		for (size_t j = i+1; j < cloud_filtered->points.size (); ++j){	
			float distance = sqrt(pow(cloud_filtered->points[j].x - cloud_filtered->points[i].x,2) + 
						pow(cloud_filtered->points[j].y - cloud_filtered->points[i].y,2) + 
						  pow(cloud_filtered->points[j].z - cloud_filtered->points[i].z,2)); 	
			for (size_t k = j+1; k < cloud_filtered->points.size (); ++k){	
				float distance2 = sqrt(pow(cloud_filtered->points[k].x - cloud_filtered->points[i].x,2) + 
					pow(cloud_filtered->points[k].y - cloud_filtered->points[i].y,2) + 
					pow(cloud_filtered->points[k].z - cloud_filtered->points[i].z,2));				
				if(fabs(distance - L12) < LAcc && fabs(distance2 - L13) < LAcc){
					P1.push_back(i);
					P2.push_back(j);
					P3.push_back(k);
		
				//ROS_INFO("at j = %d k = %d D1: %f D2: %f",j,k,distance -L1,distance2-L2);	
					}
				if(fabs(distance - L13) < LAcc && fabs(distance2 - L12) < LAcc){
					P1.push_back(i);
					P2.push_back(k);
					P3.push_back(j);

				//ROS_INFO("at j = %d k = %d D1: %f D2: %f",j,k,distance-L2,distance2-L1);
					}
				if(fabs(distance - L12) < LAcc && fabs(distance2 - L23) < LAcc){
					P1.push_back(j);
					P2.push_back(i);
					P3.push_back(k);

				//ROS_INFO("at i = %d k = %d D1: %f D2: %f",i,k,distance-L1,distance2-L3);
					}
				if(fabs(distance - L23) < LAcc && fabs(distance2 - L12) < LAcc){
					P1.push_back(k);
					P2.push_back(i);
					P3.push_back(j);
				//ROS_INFO("at i = %d j = %d D1: %f D2: %f",i,j,distance-L3,distance2-L1);
					}
				if(fabs(distance - L13) < LAcc && fabs(distance2 - L23) < LAcc){
					P1.push_back(j);
					P2.push_back(k);
					P3.push_back(i);
				//ROS_INFO("at i = %d k = %d D1: %f D2: %f",i,k,distance-L2,distance2-L3);
					}
				if(fabs(distance - L23) < LAcc && fabs(distance2 - L13) < LAcc){
					P1.push_back(k);
					P2.push_back(j);
					P3.push_back(i);
				//ROS_INFO("at i = %d j = %d D1: %f D2: %f",i,j,distance-L3,distance2-L2);
					}
				}
			}
		}
	if(P1.size() < 1 || P2.size() < 1 || P2.size() < 1) LAcc += 0.001;
	else break;
}
	//Remove duplicates
	sort( P1.begin(), P1.end());
	P1.erase( unique( P1.begin(), P1.end() ), P1.end() );
	sort( P2.begin(), P2.end());
	P2.erase( unique( P2.begin(), P2.end() ), P2.end() );
	sort( P3.begin(), P3.end());
	P3.erase( unique( P3.begin(), P3.end() ), P3.end() );
	for(cnt = 0; cnt < P1.size();++cnt)		ROS_INFO("P1Point %d: at %d: %f - %f - %f", cnt, P1.at(cnt), cloud_filtered->points[ P1.at(cnt)].x, cloud_filtered->points[ P1.at(cnt)].y, cloud_filtered->points[ P1.at(cnt)].z);

	for(cnt = 0; cnt < P2.size();++cnt)		ROS_INFO("P2Point %d: at %d: %f - %f - %f", cnt, P2.at(cnt), cloud_filtered->points[ P2.at(cnt)].x, cloud_filtered->points[ P2.at(cnt)].y, cloud_filtered->points[ P2.at(cnt)].z);
	
	for(cnt = 0; cnt < P3.size();++cnt)		ROS_INFO("P3Point %d: at %d: %f - %f - %f", cnt, P3.at(cnt), cloud_filtered->points[ P3.at(cnt)].x, cloud_filtered->points[ P3.at(cnt)].y, cloud_filtered->points[ P3.at(cnt)].z);

	float fit1, fit2;
	Eigen::Vector3f v12, v13;
	int fit_ctr = 0;	
	const int fit_size = P1.size()*P2.size()*P3.size();
	fitnessClass fitness[fit_size];
	ROS_INFO("Size: %d - %d - %d",P1.size(),P2.size(),P3.size());
	  for(cnt = 0; cnt < P1.size();cnt++){	

		for(int cnt2 = 0; cnt2 < P2.size();cnt2++){	

			for(int cnt3 = 0; cnt3 < P3.size();cnt3++){	
				v12 = vector_of(cloud_filtered->points[P1.at(cnt)],cloud_filtered->points[P2.at(cnt2)]);
				v13 = vector_of(cloud_filtered->points[P1.at(cnt)],cloud_filtered->points[P3.at(cnt3)]);
				fit1 = fabs((length_of(v12)/0.3)-1);
				fit2 = fabs((length_of(v13)/0.4)-1);
				angle = fabs((acos(dot_product(v12,v13)/(length_of(v12)*length_of(v13)))/1.5708)-1);
				fitness[fit_ctr].score = fit1+fit2+angle;
				fitness[fit_ctr].P1 = P1.at(cnt);
				fitness[fit_ctr].P2 = P2.at(cnt2);
				fitness[fit_ctr].P3 = P3.at(cnt3);
				++fit_ctr;
				ROS_INFO("Fitness: %f -- %f -- %f : %f",fit1, fit2, angle, fit1+fit2+angle);
			}
		}
	}

/*
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
		
	int p1size = P1.size();
	int p2size = P2.size();
	int p3size = P3.size();

	ROS_INFO("Location of marker (accuracy %f):",LAcc);
	ROS_INFO("P1 (%d pts): %f - %f - %f",p1size, P1x,P1y,P1z);
	ROS_INFO("P2 (%d pts): %f - %f - %f",p2size, P2x,P2y,P2z);
	ROS_INFO("P3 (%d pts): %f - %f - %f",p3size, P3x,P3y,P3z);	

	float magP12 = sqrt(pow(P2x-P1x,2)+pow(P2y-P1y,2)+pow(P2z-P1z,2));
	float magP13 = sqrt(pow(P3x-P1x,2)+pow(P3y-P1y,2)+pow(P3z-P1z,2));
	float magP23 = sqrt(pow(P3x-P2x,2)+pow(P3y-P2y,2)+pow(P3z-P2z,2));
	marker_location(0,0) = P1x;
	marker_location(0,1) = P1y;
	marker_location(0,2) = P1z;
	marker_location(1,0) = P1x + 0.3*(P2x-P1x)/magP12;
	marker_location(1,1) = P1y + 0.3*(P2y-P1y)/magP12;
	marker_location(1,2) = P1z + 0.3*(P2z-P1z)/magP12;
	marker_location(2,0) = P1x + 0.4*(P3x-P1x)/magP13;
	marker_location(2,1) = P1y + 0.4*(P3y-P1y)/magP13;
	marker_location(2,2) = P1z + 0.4*(P3z-P1z)/magP13;

/*
	P1x = marker_location(0,0);
	P1y = marker_location(0,1);
	P1z = marker_location(0,2);
	P2x = marker_location(1,0);
	P2y = marker_location(1,1);
	P2z = marker_location(1,2);
	P3x = marker_location(2,0);
	P3y = marker_location(2,1);
	P3z = marker_location(2,2);
*/

	float score = 100;
	for(fit_ctr = 0; fit_ctr<fit_size;++fit_ctr){
		if(fitness[fit_ctr].score < score){
			score = fitness[fit_ctr].score;
			P1x = cloud_filtered->points[fitness[fit_ctr].P1].x;
			P1y = cloud_filtered->points[fitness[fit_ctr].P1].y;
			P1z = cloud_filtered->points[fitness[fit_ctr].P1].z;
			P2x = cloud_filtered->points[fitness[fit_ctr].P2].x;
			P2y = cloud_filtered->points[fitness[fit_ctr].P2].y;
			P2z = cloud_filtered->points[fitness[fit_ctr].P2].z;
			P3x = cloud_filtered->points[fitness[fit_ctr].P3].x;
			P3y = cloud_filtered->points[fitness[fit_ctr].P3].y;
			P3z = cloud_filtered->points[fitness[fit_ctr].P3].z;
		}
	}

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
  pub = nh.advertise<sensor_msgs::PointCloud2> ("aligned_1", 1, true);
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_2", 1, true);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_3", 1, true);
  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_4", 1, true);
  pub4 = nh.advertise<sensor_msgs::PointCloud2> ("unaligned", 1, true);

ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker0", 0 , true);
ros::Publisher vis_pub1 = nh.advertise<visualization_msgs::Marker>( "visualization_marker1", 0 , true);
ros::Publisher vis_pub2 = nh.advertise<visualization_msgs::Marker>( "visualization_marker2", 0 , true);
ros::Publisher vis_pub3 = nh.advertise<visualization_msgs::Marker>( "visualization_marker3", 0 , true);

	marker_loc_world(0,0) = 0;
	marker_loc_world(0,1) = 0;
	marker_loc_world(0,2) = 0;
	marker_loc_world(1,0) = 0;
	marker_loc_world(1,1) = L12;
	marker_loc_world(1,2) = 0;
	marker_loc_world(2,0) = L13;
	marker_loc_world(2,1) = 0;
	marker_loc_world(2,2) = 0;

setVerbosityLevel(pcl::console::L_VERBOSE); 

  // load a file
  /*  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/oldmarker.pcd", *marker) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
*/
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/t2.pcd", *cloudA) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/t3.pcd", *cloudB) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/t2.pcd", *cloudC) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/t3.pcd", *cloudD) == -1) 
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
ROS_INFO("Locating Markers");
transformA = locate_marker(cloudA, marker_locA);
transformB = locate_marker(cloudB, marker_locB);
transformC = locate_marker(cloudC, marker_locC);
transformD = locate_marker(cloudD, marker_locD);
Eigen::Matrix4f transformAA = transformA.inverse();
Eigen::Matrix4f transformAB = transformA*transformB.inverse();
Eigen::Matrix4f transformAC = transformA*transformC.inverse();
Eigen::Matrix4f transformAD = transformA*transformD.inverse();
pcl::transformPointCloud (*cloudA, *cloudA_aligned, transformAA);
pcl::transformPointCloud (*cloudB, *cloudB_aligned2, transformAB);
pcl::transformPointCloud (*cloudC, *cloudC_aligned2, transformAC);
pcl::transformPointCloud (*cloudD, *cloudD_aligned2, transformAD);
pcl::transformPointCloud (*cloudB_aligned2, *cloudB_aligned, transformAA);
pcl::transformPointCloud (*cloudC_aligned2, *cloudC_aligned, transformAA);
pcl::transformPointCloud (*cloudD_aligned2, *cloudD_aligned, transformAA);
transformA = locate_marker(cloudA_aligned, marker_locA);
transformB = locate_marker(cloudB_aligned, marker_locB);
transformC = locate_marker(cloudC_aligned, marker_locC);
transformD = locate_marker(cloudD_aligned, marker_locD);
visualization_msgs::Marker marker1;
visualization_msgs::Marker marker2;
visualization_msgs::Marker marker3;
visualization_msgs::Marker marker4;
float m_size = 0.005;
marker1.header.frame_id = "base_link";
marker1.header.stamp = ros::Time();
marker1.ns = "my_namespace";
marker1.id = 0;
marker1.type = visualization_msgs::Marker::SPHERE_LIST;
marker1.action = visualization_msgs::Marker::ADD;

 marker1.points.resize(3);
     marker1.points[0].x = marker_locA(0,0);
     marker1.points[0].y = marker_locA(0,1);
     marker1.points[0].z = marker_locA(0,2);
     marker1.points[1].x = marker_locA(1,0);
     marker1.points[1].y = marker_locA(1,1);
     marker1.points[1].z = marker_locA(1,2);
     marker1.points[2].x = marker_locA(2,0);
     marker1.points[2].y = marker_locA(2,1);
     marker1.points[2].z = marker_locA(2,2);
marker1.scale.x = m_size;
marker1.scale.y = m_size;
marker1.scale.z = m_size;
marker1.color.a = 1.0; // Don't forget to set the alpha!
marker1.color.r = 0.0;
marker1.color.g = 1.0;
marker1.color.b = 0.0;
vis_pub.publish( marker1 );

marker2.header.frame_id = "base_link";
marker2.header.stamp = ros::Time();
marker2.ns = "my_namespace";
marker2.id = 1;
marker2.type = visualization_msgs::Marker::SPHERE_LIST;
marker2.action = visualization_msgs::Marker::ADD;

 marker2.points.resize(3);
     marker2.points[0].x = marker_locB(0,0);
     marker2.points[0].y = marker_locB(0,1);
     marker2.points[0].z = marker_locB(0,2);
     marker2.points[1].x = marker_locB(1,0);
     marker2.points[1].y = marker_locB(1,1);
     marker2.points[1].z = marker_locB(1,2);
     marker2.points[2].x = marker_locB(2,0);
     marker2.points[2].y = marker_locB(2,1);
     marker2.points[2].z = marker_locB(2,2);
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
marker3.type = visualization_msgs::Marker::SPHERE_LIST;
marker3.action = visualization_msgs::Marker::ADD;

 marker3.points.resize(3);
     marker3.points[0].x = marker_locC(0,0);
     marker3.points[0].y = marker_locC(0,1);
     marker3.points[0].z = marker_locC(0,2);
     marker3.points[1].x = marker_locC(1,0);
     marker3.points[1].y = marker_locC(1,1);
     marker3.points[1].z = marker_locC(1,2);
     marker3.points[2].x = marker_locC(2,0);
     marker3.points[2].y = marker_locC(2,1);
     marker3.points[2].z = marker_locC(2,2);
marker3.scale.x = m_size;
marker3.scale.y = m_size;
marker3.scale.z = m_size;
marker3.color.a = 1.0; // Don't forget to set the alpha!
marker3.color.r = 0.0;
marker3.color.g = 0.0;
marker3.color.b = 1.0;
vis_pub2.publish( marker3 );
marker4.header.frame_id = "base_link";
marker4.header.stamp = ros::Time();
marker4.ns = "my_namespace";
marker4.id = 3;
marker4.type = visualization_msgs::Marker::SPHERE_LIST;
marker4.action = visualization_msgs::Marker::ADD;

 marker4.points.resize(3);
     marker4.points[0].x = marker_locD(0,0);
     marker4.points[0].y = marker_locD(0,1);
     marker4.points[0].z = marker_locD(0,2);
     marker4.points[1].x = marker_locD(1,0);
     marker4.points[1].y = marker_locD(1,1);
     marker4.points[1].z = marker_locD(1,2);
     marker4.points[2].x = marker_locD(2,0);
     marker4.points[2].y = marker_locD(2,1);
     marker4.points[2].z = marker_locD(2,2);
marker4.scale.x = m_size;
marker4.scale.y = m_size;
marker4.scale.z = m_size;
marker4.color.a = 1.0; // Don't forget to set the alpha!
marker4.color.r = 1.0;
marker4.color.g = 1.0;
marker4.color.b = 0.0;
vis_pub3.publish( marker4 );

////////////////////////////////////////

/*
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
  pcl::toROSMsg(*cloudA_aligned, output5);
output5.header.frame_id = "base_link";
output5.header.stamp = ros::Time::now();
  

*/




   sensor_msgs::PointCloud2 output1;
  pcl::toROSMsg(*cloudA_aligned, output1);
output1.header.frame_id = "base_link";
output1.header.stamp = ros::Time::now();
   sensor_msgs::PointCloud2 output2;
  pcl::toROSMsg(*cloudB_aligned, output2);
output2.header.frame_id = "base_link";
output2.header.stamp = ros::Time::now();
   sensor_msgs::PointCloud2 output3;
  pcl::toROSMsg(*cloudC_aligned, output3);
output3.header.frame_id = "base_link";
output3.header.stamp = ros::Time::now();
   sensor_msgs::PointCloud2 output4;
  pcl::toROSMsg(*cloudD_aligned, output4);
output4.header.frame_id = "base_link";
output4.header.stamp = ros::Time::now();
   sensor_msgs::PointCloud2 output5;
  pcl::toROSMsg(*cloudA, output5);
output5.header.frame_id = "base_link";
output5.header.stamp = ros::Time::now();

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
  pub4.publish (output5);
ros::spinOnce();
std::cout << "Done" << std::endl;


}

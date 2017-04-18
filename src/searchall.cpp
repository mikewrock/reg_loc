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
#define m_size 0.01
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



ros::Publisher vis_pub;
ros::Publisher vis_pub1;
ros::Publisher vis_pub2;
visualization_msgs::Marker marker1;
visualization_msgs::Marker marker2;
visualization_msgs::Marker marker3;

float leaf_size;
float LACC, DOT, LACC2;
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

DOT = config.dot_product;
LACC = config.dot_product_accuracy;
LACC2 = config.purpendicularity_accuracy;
}


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

bool locate_marker(std::vector<Eigen::Vector4f>& planes_loc, Eigen::Matrix3f &marker_location, Eigen::Matrix4f &A_mat){

	//ROS_INFO("Locating %d planes", planes_loc.size());
	int cnts = 0;
	int i = 0;
	int j = 0;
	int k = 0;
	bool marker_flag = false;
	Eigen::Vector3f intersection, holder;
	Eigen::Vector3f best_intersection(100,100,100);
	float best_value = 100;
	float value;
	Eigen::Vector3f purpendicular,parallel1, parallel2;
	float P1x;
	float P1y;
	float P1z;
	float P2x;
	float P2y;
	float P2z;
	float P3x;
	float P3y;
	float P3z;
	Eigen::Vector3f a,n,c,o;
	float mn, mc;

	while( i < (planes_loc.size()-1) && planes_loc.size() >= 2 ){
		j = i+1; 
		while(  j < (planes_loc.size())){
				value = check_planes(planes_loc.at(i),planes_loc.at(j));
				if(fabs(fabs(value)-DOT) < best_value){

					intersection(2) = 0; 	
					intersection(1) = (-planes_loc.at(i)(3) + (planes_loc.at(i)(0)/planes_loc.at(j)(0))*planes_loc.at(j)(3))/(planes_loc.at(i)(1)-(planes_loc.at(i)(0)/planes_loc.at(j)(0))*planes_loc.at(j)(1));
					intersection(0)	= (-planes_loc.at(i)(1)*intersection(1)-planes_loc.at(i)(3))/planes_loc.at(i)(0);
					purpendicular(0) = 0;
					purpendicular(1) = 0;
					purpendicular(2) = 1;
					holder = normalized_cross_product(planes_loc.at(i),planes_loc.at(j));
					if(holder(2) > 0){
						if(value > 0){
							parallel1 = normalized_cross_product(purpendicular,planes_loc.at(i));
							parallel2 = normalized_cross_product(purpendicular,planes_loc.at(j));
						}

						else{
							parallel2 = normalized_cross_product(planes_loc.at(i),purpendicular);
							parallel1 = normalized_cross_product(purpendicular,planes_loc.at(j));
						}
					}

					else{
						if(value > 0) {
							parallel2 = normalized_cross_product(planes_loc.at(i),purpendicular);
							parallel1 = normalized_cross_product(planes_loc.at(j),purpendicular);
						}
						else	{
							parallel1 = normalized_cross_product(purpendicular,planes_loc.at(i));
							parallel2 = normalized_cross_product(planes_loc.at(j),purpendicular);
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


	if(marker_flag == true){

		 P1x = intersection(0);
		 P1y = intersection(1);
		 P1z = intersection(2);
		 P2x = intersection(0) + parallel1(0);
		 P2y = intersection(1) + parallel1(1);
		 P2z = intersection(2) + parallel1(2);
		 P3x = intersection(0) + parallel2(0);
		 P3y = intersection(1) + parallel2(1);
		 P3z = intersection(2) + parallel2(2);
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

		mn = sqrt(pow(P3x-P2x,2)+pow(P3y-P2y,2)+pow(P3z-P2z,2));
		mc = sqrt(pow(P1x-P2x,2)+pow(P1y-P2y,2)+pow(P1z-P2z,2));
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

}
	else ROS_INFO("Could not find marker");


	return marker_flag;

}



Eigen::Vector4f refine_plane( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){
sensor_msgs::PointCloud2 output2;
	Eigen::Vector4f plane;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZI> extract;

    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg2; 
  seg2.setOptimizeCoefficients (optimize_coefficients2);

  // Mandatory
  seg2.setModelType (11);
  seg2.setMethodType (pcl::SAC_RANSAC);
  seg2.setMaxIterations (max_iterations);
  seg2.setDistanceThreshold (distance_threshold2);
  seg2.setNormalDistanceWeight(normal_distance_weight2);
  seg2.setEpsAngle(eps_angle);

    seg2.setInputCloud (cloud_src);
    seg2.setInputNormals(cloud_normals); 
    seg2.segment (*inliers, *coefficients);

    // Extract the inliers
    extract.setInputCloud (cloud_src);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
    std::cerr << "PointCloud representing the refined planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
	plane(0) = coefficients->values[0];
	plane(1) = coefficients->values[1];
	plane(2) = coefficients->values[2];
	plane(3) = coefficients->values[3];

		pcl::toROSMsg(*cloud_p, output2);
		output2.header.frame_id = "base_link";
		output2.header.stamp = ros::Time::now();
		pub2.publish(output2);
	return plane;

}

int
main (int argc, char** argv)
{
// Initialize ROS
ros::init (argc, argv, "my_pcl_tutorial");


ros::NodeHandle nh;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZI> );
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_save (new pcl::PointCloud<pcl::PointXYZI>);
// Create a ROS publisher for the output point cloud
pub = nh.advertise<sensor_msgs::PointCloud2> ("aligned_1", 1, true);
pub1 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_2", 1, true);
pub2 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_3", 1, true);
pub3 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_4", 1, true);
pub4 = nh.advertise<sensor_msgs::PointCloud2> ("aligned_5", 1, true);
sensor_msgs::PointCloud2 output, output1, output3, output2, output4;
vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker0", 0 , true);
vis_pub1 = nh.advertise<visualization_msgs::Marker>( "visualization_marker1", 0 , true);
vis_pub2 = nh.advertise<visualization_msgs::Marker>( "visualization_marker2", 0 , true);

dynamic_reconfigure::Server<registration_localization::SEGConfig> server;
dynamic_reconfigure::Server<registration_localization::SEGConfig>::CallbackType f;

f = boost::bind(&callback, _1, _2);
server.setCallback(f);

std_msgs::String filename;    
std::stringstream ss;
ss << "/home/mike/marker/" << argv[1] << "1.pcd";
if (pcl::io::loadPCDFile<pcl::PointXYZI> (ss.str(), *cloudA) == -1) 
{
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

	return(0);
}


std::cerr << "PointCloud before filtering: " << cloudA->width * cloudA->height << " data points." << std::endl;
pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>), cloud_p (new pcl::PointCloud<pcl::PointXYZI>), cloud_f (new pcl::PointCloud<pcl::PointXYZI>);
Eigen::Vector4f plane, plane_r;
std::vector<Eigen::Vector4f> planes;
std::vector<Eigen::Vector4f> transforms;

Eigen::Matrix4f transformA;
Eigen::Matrix4f transformB;
Eigen::Matrix4f transformC;
Eigen::Matrix4f transformD;
Eigen::Matrix4f transformAA;
Eigen::Matrix4f transformAB;
Eigen::Matrix4f transformAC;
Eigen::Matrix4f transformAD;

bool finished = false;
while(ros::ok() && finished == false){
	reco = false;
	planes.clear();
	//Publish initial cloud
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
	//donsample cloud
	if(downsample){
	ROS_INFO("Skipping downsample");
	  cloud_filtered.swap(cloudA);
	  std::cerr << "using " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
	}	
	//publish downsampled coud
	pcl::toROSMsg(*cloud_filtered, output1);
	output1.header.frame_id = "base_link";
	output1.header.stamp = ros::Time::now();
	pub1.publish(output1);
	ros::spinOnce();

	//prepare segmentation
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	  // Create the segmentation object
	    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg; 
	  // Optional
	  seg.setOptimizeCoefficients (optimize_coefficients);
	  // Mandatory
	  seg.setModelType (11);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (max_iterations);
	  seg.setDistanceThreshold (distance_threshold);
	  seg.setNormalDistanceWeight(normal_distance_weight);
	  seg.setEpsAngle(eps_angle);
	  // Create the filtering object
	  pcl::ExtractIndices<pcl::PointXYZI> extract;
	  pcl::ExtractIndices<pcl::Normal> extract_normals;
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
	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f (new pcl::PointCloud<pcl::Normal>);
	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_p (new pcl::PointCloud<pcl::Normal>);
	  // Use all neighbors in a sphere of radius 3cm
	  ne.setRadiusSearch (radius_search);
	  ne.setKSearch (k_search);
	  // Compute the features
	  ne.compute (*cloud_normals);

	//start looking through planes
	int ctr = 0;
	while(ctr < 30 && reco == false && ros::ok() && finished == false){
		++ctr;


		ROS_INFO("Iteration %d",ctr);

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

		extract_normals.setInputCloud (cloud_normals);
		extract_normals.setIndices (inliers); 
		extract_normals.setNegative (false);
		extract_normals.filter (*cloud_normals_p);
    		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		
		plane(0) = coefficients->values[0];
		plane(1) = coefficients->values[1];
		plane(2) = coefficients->values[2];
		plane(3) = coefficients->values[3];

		pcl::toROSMsg(*cloud_p, output3);
		output3.header.frame_id = "base_link";
		output3.header.stamp = ros::Time::now();

		plane_r = refine_plane(cloud_p,cloud_normals_p);

		planes.push_back(plane_r);
		Eigen::Matrix3f marker_loc;
		Eigen::Matrix4f transform_mat;
		if(locate_marker(planes, marker_loc, transform_mat )){
			
			transformA = transform_mat;
			marker1.header.frame_id = "base_link";
			marker1.header.stamp = ros::Time();
			marker1.ns = "my_namespace";
			marker1.id = 0;
			marker1.type = visualization_msgs::Marker::SPHERE;
			marker1.action = visualization_msgs::Marker::ADD;
			marker1.pose.position.x = marker_loc(0,0);
			marker1.pose.position.y = marker_loc(0,1);
			marker1.pose.position.z = marker_loc(0,2);
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
			marker2.points[0].x = marker_loc(0,0);
			marker2.points[0].y = marker_loc(0,1);
			marker2.points[0].z = marker_loc(0,2);
			marker2.points[1].x = marker_loc(1,0);
			marker2.points[1].y = marker_loc(1,1);
			marker2.points[1].z = marker_loc(1,2);
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
			marker3.points[0].x = marker_loc(0,0);
			marker3.points[0].y = marker_loc(0,1);
			marker3.points[0].z = marker_loc(0,2);
			marker3.points[1].x = marker_loc(2,0);
			marker3.points[1].y = marker_loc(2,1);
			marker3.points[1].z = marker_loc(2,2);
			marker3.scale.x = m_size;
			marker3.scale.y = m_size;
			marker3.scale.z = m_size;
			marker3.color.a = 1.0; // Don't forget to set the alpha!
			marker3.color.r = 0.0;
			marker3.color.g = 0.0;
			marker3.color.b = 1.0;
			vis_pub2.publish( marker3 );
			ros::spinOnce();
			finished = true;

		}
		//extract remining points
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered.swap (cloud_f);
		extract_normals.setNegative (true);
		extract_normals.filter (*cloud_normals_f);
		cloud_normals.swap (cloud_normals_f);


		pcl::toROSMsg(*cloud_filtered, output1);
		output1.header.frame_id = "base_link";
		output1.header.stamp = ros::Time::now();
		pub1.publish(output1);
		pub3.publish(output3);
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();








		}
	}

ss.str(std::string());
ss << "/home/mike/marker/" << argv[1] << "2.pcd";
if (pcl::io::loadPCDFile<pcl::PointXYZI> (ss.str(), *cloudA) == -1) 
{
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

	return(0);
}
ROS_INFO("Starting cloud 2");
finished = false;
while(ros::ok() && finished == false){
	reco = false;
	planes.clear();
	//Publish initial cloud
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
	//donsample cloud
	if(downsample){
	ROS_INFO("Skipping downsample");
	  cloud_filtered.swap(cloudA);
	  std::cerr << "using " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
	}	
	//publish downsampled coud
	pcl::toROSMsg(*cloud_filtered, output1);
	output1.header.frame_id = "base_link";
	output1.header.stamp = ros::Time::now();
	pub1.publish(output1);
	ros::spinOnce();

	//prepare segmentation
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	  // Create the segmentation object
	    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg; 
	  // Optional
	  seg.setOptimizeCoefficients (optimize_coefficients);
	  // Mandatory
	  seg.setModelType (11);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (max_iterations);
	  seg.setDistanceThreshold (distance_threshold);
	  seg.setNormalDistanceWeight(normal_distance_weight);
	  seg.setEpsAngle(eps_angle);
	  // Create the filtering object
	  pcl::ExtractIndices<pcl::PointXYZI> extract;
	  pcl::ExtractIndices<pcl::Normal> extract_normals;
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
	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f (new pcl::PointCloud<pcl::Normal>);
	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_p (new pcl::PointCloud<pcl::Normal>);
	  // Use all neighbors in a sphere of radius 3cm
	  ne.setRadiusSearch (radius_search);
	  ne.setKSearch (k_search);
	  // Compute the features
	  ne.compute (*cloud_normals);

	//start looking through planes
	int ctr = 0;
	while(ctr < 30 && reco == false && ros::ok() &&finished == false){
		++ctr;


		ROS_INFO("Iteration %d",ctr);

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

		extract_normals.setInputCloud (cloud_normals);
		extract_normals.setIndices (inliers); 
		extract_normals.setNegative (false);
		extract_normals.filter (*cloud_normals_p);
    		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		
		plane(0) = coefficients->values[0];
		plane(1) = coefficients->values[1];
		plane(2) = coefficients->values[2];
		plane(3) = coefficients->values[3];

		pcl::toROSMsg(*cloud_p, output3);
		output3.header.frame_id = "base_link";
		output3.header.stamp = ros::Time::now();

		plane_r = refine_plane(cloud_p,cloud_normals_p);

		planes.push_back(plane_r);
		Eigen::Matrix3f marker_loc;
		Eigen::Matrix4f transform_mat;
		if(locate_marker(planes, marker_loc, transform_mat )){
			

			transformB = transform_mat;
			marker1.header.frame_id = "base_link";
			marker1.header.stamp = ros::Time();
			marker1.ns = "my_namespace";
			marker1.id = 0;
			marker1.type = visualization_msgs::Marker::SPHERE;
			marker1.action = visualization_msgs::Marker::ADD;
			marker1.pose.position.x = marker_loc(0,0);
			marker1.pose.position.y = marker_loc(0,1);
			marker1.pose.position.z = marker_loc(0,2);
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
			marker2.points[0].x = marker_loc(0,0);
			marker2.points[0].y = marker_loc(0,1);
			marker2.points[0].z = marker_loc(0,2);
			marker2.points[1].x = marker_loc(1,0);
			marker2.points[1].y = marker_loc(1,1);
			marker2.points[1].z = marker_loc(1,2);
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
			marker3.points[0].x = marker_loc(0,0);
			marker3.points[0].y = marker_loc(0,1);
			marker3.points[0].z = marker_loc(0,2);
			marker3.points[1].x = marker_loc(2,0);
			marker3.points[1].y = marker_loc(2,1);
			marker3.points[1].z = marker_loc(2,2);
			marker3.scale.x = m_size;
			marker3.scale.y = m_size;
			marker3.scale.z = m_size;
			marker3.color.a = 1.0; // Don't forget to set the alpha!
			marker3.color.r = 0.0;
			marker3.color.g = 0.0;
			marker3.color.b = 1.0;
			vis_pub2.publish( marker3 );
			ros::spinOnce();
			finished = true;

		}
		//extract remining points
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered.swap (cloud_f);
		extract_normals.setNegative (true);
		extract_normals.filter (*cloud_normals_f);
		cloud_normals.swap (cloud_normals_f);


		pcl::toROSMsg(*cloud_filtered, output1);
		output1.header.frame_id = "base_link";
		output1.header.stamp = ros::Time::now();
		pub1.publish(output1);
		pub3.publish(output3);
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();








		}
	}

 ss.str(std::string());
ss << "/home/mike/marker/" << argv[1] << "3.pcd";
if (pcl::io::loadPCDFile<pcl::PointXYZI> (ss.str(), *cloudA) == -1) 
{
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

	return(0);
}

ROS_INFO("Starting cloud 3");
finished = false;
while(ros::ok() && finished == false){
	reco = false;
	planes.clear();
	//Publish initial cloud
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
	//donsample cloud
	if(downsample){
	ROS_INFO("Skipping downsample");
	  cloud_filtered.swap(cloudA);
	  std::cerr << "using " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
	}	
	//publish downsampled coud
	pcl::toROSMsg(*cloud_filtered, output1);
	output1.header.frame_id = "base_link";
	output1.header.stamp = ros::Time::now();
	pub1.publish(output1);
	ros::spinOnce();

	//prepare segmentation
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	  // Create the segmentation object
	    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg; 
	  // Optional
	  seg.setOptimizeCoefficients (optimize_coefficients);
	  // Mandatory
	  seg.setModelType (11);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (max_iterations);
	  seg.setDistanceThreshold (distance_threshold);
	  seg.setNormalDistanceWeight(normal_distance_weight);
	  seg.setEpsAngle(eps_angle);
	  // Create the filtering object
	  pcl::ExtractIndices<pcl::PointXYZI> extract;
	  pcl::ExtractIndices<pcl::Normal> extract_normals;
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
	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f (new pcl::PointCloud<pcl::Normal>);
	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_p (new pcl::PointCloud<pcl::Normal>);
	  // Use all neighbors in a sphere of radius 3cm
	  ne.setRadiusSearch (radius_search);
	  ne.setKSearch (k_search);
	  // Compute the features
	  ne.compute (*cloud_normals);

	//start looking through planes
	int ctr = 0;
	while(ctr < 30 && reco == false && ros::ok() &&finished == false){
		++ctr;


		ROS_INFO("Iteration %d",ctr);

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

		extract_normals.setInputCloud (cloud_normals);
		extract_normals.setIndices (inliers); 
		extract_normals.setNegative (false);
		extract_normals.filter (*cloud_normals_p);
    		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		
		plane(0) = coefficients->values[0];
		plane(1) = coefficients->values[1];
		plane(2) = coefficients->values[2];
		plane(3) = coefficients->values[3];

		pcl::toROSMsg(*cloud_p, output3);
		output3.header.frame_id = "base_link";
		output3.header.stamp = ros::Time::now();

		plane_r = refine_plane(cloud_p,cloud_normals_p);

		planes.push_back(plane_r);
		Eigen::Matrix3f marker_loc;
		Eigen::Matrix4f transform_mat;
		if(locate_marker(planes, marker_loc, transform_mat )){
			

			transformC = transform_mat;
			marker1.header.frame_id = "base_link";
			marker1.header.stamp = ros::Time();
			marker1.ns = "my_namespace";
			marker1.id = 0;
			marker1.type = visualization_msgs::Marker::SPHERE;
			marker1.action = visualization_msgs::Marker::ADD;
			marker1.pose.position.x = marker_loc(0,0);
			marker1.pose.position.y = marker_loc(0,1);
			marker1.pose.position.z = marker_loc(0,2);
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
			marker2.points[0].x = marker_loc(0,0);
			marker2.points[0].y = marker_loc(0,1);
			marker2.points[0].z = marker_loc(0,2);
			marker2.points[1].x = marker_loc(1,0);
			marker2.points[1].y = marker_loc(1,1);
			marker2.points[1].z = marker_loc(1,2);
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
			marker3.points[0].x = marker_loc(0,0);
			marker3.points[0].y = marker_loc(0,1);
			marker3.points[0].z = marker_loc(0,2);
			marker3.points[1].x = marker_loc(2,0);
			marker3.points[1].y = marker_loc(2,1);
			marker3.points[1].z = marker_loc(2,2);
			marker3.scale.x = m_size;
			marker3.scale.y = m_size;
			marker3.scale.z = m_size;
			marker3.color.a = 1.0; // Don't forget to set the alpha!
			marker3.color.r = 0.0;
			marker3.color.g = 0.0;
			marker3.color.b = 1.0;
			vis_pub2.publish( marker3 );
			ros::spinOnce();
			finished = true;

		}
		//extract remining points
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered.swap (cloud_f);
		extract_normals.setNegative (true);
		extract_normals.filter (*cloud_normals_f);
		cloud_normals.swap (cloud_normals_f);


		pcl::toROSMsg(*cloud_filtered, output1);
		output1.header.frame_id = "base_link";
		output1.header.stamp = ros::Time::now();
		pub1.publish(output1);
		pub3.publish(output3);
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();








		}
	}



ss.str(std::string());
ss << "/home/mike/marker/" << argv[1] << "4.pcd";
if (pcl::io::loadPCDFile<pcl::PointXYZI> (ss.str(), *cloudA) == -1) 
{
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

	return(0);
}

ROS_INFO("Starting cloud 4");
finished = false;
while(ros::ok() && finished == false){
	reco = false;
	planes.clear();
	//Publish initial cloud
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
	//donsample cloud
	if(downsample){
	ROS_INFO("Skipping downsample");
	  cloud_filtered.swap(cloudA);
	  std::cerr << "using " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
	}	
	//publish downsampled coud
	pcl::toROSMsg(*cloud_filtered, output1);
	output1.header.frame_id = "base_link";
	output1.header.stamp = ros::Time::now();
	pub1.publish(output1);
	ros::spinOnce();

	//prepare segmentation
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	  // Create the segmentation object
	    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg; 
	  // Optional
	  seg.setOptimizeCoefficients (optimize_coefficients);
	  // Mandatory
	  seg.setModelType (11);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (max_iterations);
	  seg.setDistanceThreshold (distance_threshold);
	  seg.setNormalDistanceWeight(normal_distance_weight);
	  seg.setEpsAngle(eps_angle);
	  // Create the filtering object
	  pcl::ExtractIndices<pcl::PointXYZI> extract;
	  pcl::ExtractIndices<pcl::Normal> extract_normals;
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
	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f (new pcl::PointCloud<pcl::Normal>);
	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_p (new pcl::PointCloud<pcl::Normal>);
	  // Use all neighbors in a sphere of radius 3cm
	  ne.setRadiusSearch (radius_search);
	  ne.setKSearch (k_search);
	  // Compute the features
	  ne.compute (*cloud_normals);

	//start looking through planes
	int ctr = 0;
	while(ctr < 30 && reco == false && ros::ok() &&finished == false){
		++ctr;


		ROS_INFO("Iteration %d",ctr);

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

		extract_normals.setInputCloud (cloud_normals);
		extract_normals.setIndices (inliers); 
		extract_normals.setNegative (false);
		extract_normals.filter (*cloud_normals_p);
    		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		
		plane(0) = coefficients->values[0];
		plane(1) = coefficients->values[1];
		plane(2) = coefficients->values[2];
		plane(3) = coefficients->values[3];

		pcl::toROSMsg(*cloud_p, output3);
		output3.header.frame_id = "base_link";
		output3.header.stamp = ros::Time::now();

		plane_r = refine_plane(cloud_p,cloud_normals_p);

		planes.push_back(plane_r);
		Eigen::Matrix3f marker_loc;
		Eigen::Matrix4f transform_mat;
		if(locate_marker(planes, marker_loc, transform_mat )){
			
			
			transformD = transform_mat;
			marker1.header.frame_id = "base_link";
			marker1.header.stamp = ros::Time();
			marker1.ns = "my_namespace";
			marker1.id = 0;
			marker1.type = visualization_msgs::Marker::SPHERE;
			marker1.action = visualization_msgs::Marker::ADD;
			marker1.pose.position.x = marker_loc(0,0);
			marker1.pose.position.y = marker_loc(0,1);
			marker1.pose.position.z = marker_loc(0,2);
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
			marker2.points[0].x = marker_loc(0,0);
			marker2.points[0].y = marker_loc(0,1);
			marker2.points[0].z = marker_loc(0,2);
			marker2.points[1].x = marker_loc(1,0);
			marker2.points[1].y = marker_loc(1,1);
			marker2.points[1].z = marker_loc(1,2);
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
			marker3.points[0].x = marker_loc(0,0);
			marker3.points[0].y = marker_loc(0,1);
			marker3.points[0].z = marker_loc(0,2);
			marker3.points[1].x = marker_loc(2,0);
			marker3.points[1].y = marker_loc(2,1);
			marker3.points[1].z = marker_loc(2,2);
			marker3.scale.x = m_size;
			marker3.scale.y = m_size;
			marker3.scale.z = m_size;
			marker3.color.a = 1.0; // Don't forget to set the alpha!
			marker3.color.r = 0.0;
			marker3.color.g = 0.0;
			marker3.color.b = 1.0;
			vis_pub2.publish( marker3 );
			ros::spinOnce();
			finished = true;

		}
		//extract remining points
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered.swap (cloud_f);
		extract_normals.setNegative (true);
		extract_normals.filter (*cloud_normals_f);
		cloud_normals.swap (cloud_normals_f);


		pcl::toROSMsg(*cloud_filtered, output1);
		output1.header.frame_id = "base_link";
		output1.header.stamp = ros::Time::now();
		pub1.publish(output1);
		pub3.publish(output3);
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();








		}
	}


transformAA = transformA.inverse();
transformAB = transformA*transformB.inverse();
transformAC = transformA*transformC.inverse();
transformAD = transformA*transformD.inverse();
pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_output (new pcl::PointCloud<pcl::PointXYZI> );

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZI> );

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC (new pcl::PointCloud<pcl::PointXYZI> );

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD (new pcl::PointCloud<pcl::PointXYZI> );

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_int (new pcl::PointCloud<pcl::PointXYZI> );

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC_int (new pcl::PointCloud<pcl::PointXYZI> );

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD_int (new pcl::PointCloud<pcl::PointXYZI> );
pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_output (new pcl::PointCloud<pcl::PointXYZI> );

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC_output (new pcl::PointCloud<pcl::PointXYZI> );

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD_output (new pcl::PointCloud<pcl::PointXYZI> );
ss.str(std::string());
ss << "/home/mike/marker/" << argv[1] << "1.pcd";
if (pcl::io::loadPCDFile<pcl::PointXYZI> (ss.str(), *cloudA) == -1) 
{
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

	return(0);
}
ss.str(std::string());
ss << "/home/mike/marker/" << argv[1] << "2.pcd";
if (pcl::io::loadPCDFile<pcl::PointXYZI> (ss.str(), *cloudB) == -1) 
{
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

	return(0);
}
ss.str(std::string());
ss << "/home/mike/marker/" << argv[1] << "3.pcd";
if (pcl::io::loadPCDFile<pcl::PointXYZI> (ss.str(), *cloudC) == -1) 
{
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

	return(0);
}
ss.str(std::string());
ss << "/home/mike/marker/" << argv[1] << "4.pcd";
if (pcl::io::loadPCDFile<pcl::PointXYZI> (ss.str(), *cloudD) == -1) 
{
	PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

	return(0);
}




pcl::transformPointCloud (*cloudA, *cloudA_output, transformAA);
pcl::transformPointCloud (*cloudB, *cloudB_int, transformAB);
pcl::transformPointCloud (*cloudB_int, *cloudB_output, transformAA);
pcl::transformPointCloud (*cloudC, *cloudC_int, transformAC);
pcl::transformPointCloud (*cloudC_int, *cloudC_output, transformAA);
pcl::transformPointCloud (*cloudD, *cloudD_int, transformAD);
pcl::transformPointCloud (*cloudD_int, *cloudD_output, transformAA);
		pcl::toROSMsg(*cloudA_output, output1);
		output1.header.frame_id = "base_link";
		output1.header.stamp = ros::Time::now();
		pub1.publish(output1);
		pcl::toROSMsg(*cloudB_output, output2);
		output2.header.frame_id = "base_link";
		output2.header.stamp = ros::Time::now();
		pub2.publish(output2);
		pcl::toROSMsg(*cloudC_output, output3);
		output3.header.frame_id = "base_link";
		output3.header.stamp = ros::Time::now();
		pub3.publish(output3);
		pcl::toROSMsg(*cloudD_output, output4);
		output4.header.frame_id = "base_link";
		output4.header.stamp = ros::Time::now();
		pub4.publish(output4);

std::cout << "done" << std::endl;




}

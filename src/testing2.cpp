#include <ros/ros.h>
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
#include <Eigen/Core>
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
 int cloudctr = 0;

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.01f;
float support_size = 0.001f;
float support_size2 = 0.001f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

//convenient typedefs
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD (new pcl::PointCloud<pcl::PointXYZI> );
//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};





class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZI> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.01f),
      feature_radius_ (0.01f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (const PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.01f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (500)
    {
      // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputSource (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZI> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZI, pcl::PointXYZI, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};





////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
/////Experimentation////////////
/*
 // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (cloud_tgt); 
// Assign to the source FeatureCloud
  FeatureCloud source_cloud;
  source_cloud.setInputCloud (cloud_src);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
    template_align.addTemplateCloud (source_cloud);
  template_align.setTargetCloud (target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment (best_alignment);

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

/////////////////////////////////////////////////

*/

setVerbosityLevel(pcl::console::L_VERBOSE); 
//Create a PCL pointcloud
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>& point_cloud = *cloud_src;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());

    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>& point_cloud2 = *cloud_tgt;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges2;
  Eigen::Affine3f scene_sensor_pose2 (Eigen::Affine3f::Identity ());

    scene_sensor_pose2 = Eigen::Affine3f (Eigen::Translation3f (point_cloud2.sensor_origin_[0],
                                                               point_cloud2.sensor_origin_[1],
                                                               point_cloud2.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud2.sensor_orientation_);
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
    range_image.setUnseenToMaxRange ();
   std::cout << range_image << "\n";
  boost::shared_ptr<pcl::RangeImage> range_image_ptr2 (new pcl::RangeImage);
  pcl::RangeImage& range_image2 = *range_image_ptr2;   
  range_image2.createFromPointCloud (point_cloud2, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose2, coordinate_frame, noise_level, min_range, border_size);
  range_image2.integrateFarRanges (far_ranges2);
    range_image2.setUnseenToMaxRange ();
   std::cout << range_image2 << "\n";
  
  // --------------------------------
  // -----Extract NARF keypoints-----
  // --------------------------------
  pcl::RangeImageBorderExtractor range_image_border_extractor;
  pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage (&range_image);
  narf_keypoint_detector.getParameters ().support_size = support_size;
  //narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
  //narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;
  pcl::RangeImageBorderExtractor range_image_border_extractor2;
  pcl::NarfKeypoint narf_keypoint_detector2 (&range_image_border_extractor2);
  narf_keypoint_detector2.setRangeImage (&range_image2);
  narf_keypoint_detector2.getParameters ().support_size = support_size;
  //narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
  //narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;
  
  pcl::PointCloud<int> keypoint_indices;
  narf_keypoint_detector.compute (keypoint_indices);
  std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";
  pcl::PointCloud<int> keypoint_indices2;
  narf_keypoint_detector2.compute (keypoint_indices2);
  std::cout << "Found2 "<<keypoint_indices2.points.size ()<<" key points.\n";

  
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
  keypoints.points.resize (keypoint_indices.points.size ());
  for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& keypoints2 = *keypoints_ptr2;
  keypoints2.points.resize (keypoint_indices2.points.size ());
  for (size_t i=0; i<keypoint_indices2.points.size (); ++i)
    keypoints2.points[i].getVector3fMap () = range_image2.points[keypoint_indices2.points[i]].getVector3fMap ();


  std::vector<int> keypoint_indices2x;
  keypoint_indices2x.resize (keypoint_indices.points.size ());
  for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
    keypoint_indices2x[i]=keypoint_indices.points[i];
  pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2x);
  narf_descriptor.getParameters ().support_size = support_size;
  narf_descriptor.getParameters ().rotation_invariant = false;
  pcl::PointCloud<pcl::Narf36> narf_descriptors;
  narf_descriptor.compute (narf_descriptors);
  cout << "Extracted "<<narf_descriptors.size ()<<" descriptors for "
                      <<keypoint_indices.points.size ()<< " keypoints.\n";


  std::vector<int> keypoint_indices2y;
  keypoint_indices2y.resize (keypoint_indices2.points.size ());
  for (unsigned int i=0; i<keypoint_indices2.size (); ++i) // This step is necessary to get the right vector type
    keypoint_indices2y[i]=keypoint_indices2.points[i];
  pcl::NarfDescriptor narf_descriptor2 (&range_image2, &keypoint_indices2y);
  narf_descriptor2.getParameters ().support_size = support_size2;
  narf_descriptor2.getParameters ().rotation_invariant = false;
  pcl::PointCloud<pcl::Narf36> narf_descriptors2;
  narf_descriptor2.compute (narf_descriptors2);
  cout << "Extracted2 "<<narf_descriptors2.size ()<<" descriptors for "
                      <<keypoint_indices2.points.size ()<< " keypoints.\n";








pcl::registration::CorrespondenceEstimation<pcl::Narf36, pcl::Narf36> reg; 
pcl::CorrespondencesPtr all_correspondences_ptr(new pcl::Correspondences); 
pcl::Correspondences& all_correspondences = *all_correspondences_ptr; 
pcl::Correspondences final_correspondences; 
boost::shared_ptr<pcl::PointCloud<pcl::Narf36> > src_narf_descriptors_ptr(new pcl::PointCloud<pcl::Narf36>(narf_descriptors)); 
reg.setInputSource(src_narf_descriptors_ptr); 
boost::shared_ptr<pcl::PointCloud<pcl::Narf36> > tgt_narf_descriptors_ptr(new pcl::PointCloud<pcl::Narf36>(narf_descriptors2)); 
reg.setInputTarget(tgt_narf_descriptors_ptr); 
reg.determineReciprocalCorrespondences(all_correspondences); 
cout << "Found "<<all_correspondences.size()<<" total feature correspondences.\n"; 


pcl::registration::CorrespondenceRejectorDistance cRejDist; 
cRejDist.setInputSource<pcl::PointXYZ>(keypoints_ptr); 
cRejDist.setInputTarget<pcl::PointXYZ>(keypoints_ptr2); 
cRejDist.setMaximumDistance(0.5); 
cRejDist.setInputCorrespondences(all_correspondences_ptr); 
cRejDist.getCorrespondences(final_correspondences); 

pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est; 
Eigen::Matrix4f trans, ttsTrans; // ttsTrans: target to source transformation 
trans_est.estimateRigidTransformation(keypoints, keypoints2, all_correspondences, trans); 
ttsTrans = trans.inverse(); 

cout << ttsTrans;
/*
      pcl::PointCloud<pcl::Normal>::Ptr normals_ = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);

      pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> norm_est;
      norm_est.setInputCloud (cloud_src);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr search_method_xyz_;
      norm_est.setSearchMethod (search_method_xyz_);

      norm_est.setRadiusSearch (0.01f);
      norm_est.compute (*normals_);
cout << "Nsize: " << normals_->points.size();

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_ = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new pcl::PointCloud<pcl::FPFHSignature33>);

      pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (cloud_src);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (0.01f);
      fpfh_est.compute (*features_);

cout << "Fsize: " << features_->points.size() ;

      pcl::PointCloud<pcl::Normal>::Ptr tgt_normals_ = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);

      pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> tgt_norm_est;
      norm_est.setInputCloud (cloud_tgt);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tgt_search_method_xyz_;
      norm_est.setSearchMethod (tgt_search_method_xyz_);

      norm_est.setRadiusSearch (0.01f);
      norm_est.compute (*tgt_normals_);

cout << "Nsize: " << tgt_normals_->points.size();

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgt_features_ = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new pcl::PointCloud<pcl::FPFHSignature33>);

      pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> tgt_fpfh_est;
      fpfh_est.setInputCloud (cloud_tgt);
      fpfh_est.setInputNormals (tgt_normals_);
      fpfh_est.setSearchMethod (tgt_search_method_xyz_);
      fpfh_est.setRadiusSearch (0.01f);
      fpfh_est.compute (*tgt_features_);


cout << "Fsize: " << tgt_features_->points.size();

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZI, pcl::PointXYZI, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_ = 0.01f;
    float max_correspondence_distance_ = 0.001f*0.001f;
    int nr_iterations_ = 300;

      sac_ia_.setInputTarget (cloud_tgt);
      sac_ia_.setTargetFeatures (tgt_features_);


      sac_ia_.setInputSource (cloud_src);
      sac_ia_.setSourceFeatures (features_);

pcl::PointCloud<pcl::PointXYZI>::Ptr reg_ptr (new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>& registration_output = *reg_ptr;



      sac_ia_.align (registration_output);
      float fitness_score = (float) sac_ia_.getFitnessScore (0.01f);
      Eigen::Matrix4f initial_transformation= sac_ia_.getFinalTransformation ();

	printf("Fitness: %f\n", fitness_score);
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  // Get the transformation from target to source
  targetToSource = initial_transformation.inverse();
cout << targetToSource;





pcl::transformPointCloud (*cloud_src, *output, trans);

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setUseReciprocalCorrespondences(true);
  icp.setInputSource (output);
  icp.setInputTarget (cloud_tgt);
  typedef pcl::registration::TransformationEstimationLM <pcl::PointXYZI, pcl::PointXYZI> te;
  boost::shared_ptr<te> teLM (new te);
 icp.setTransformationEstimation (teLM);
  icp.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  icp.setMaxCorrespondenceDistance (.01); 
pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>::Ptr ce (new pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>);
//ce->setInputSource (cloud_src);
//ce->setInputTarget (cloud_tgt);
////icp.setCorrespondenceEstimation (ce);
 //icp.setTransformationEpsilon (.1);
//icp.setEuclideanFitnessEpsilon(.01);
icp.setMaximumIterations(1000);
  icp.setRANSACIterations(100);
//icp.setRANSACOutlierRejectionThreshold (.02);
  pcl::PointCloud<pcl::PointXYZI> Final;
  icp.align (Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  std::cout << "max iterations" << icp.getMaximumIterations() << " ransac: " << icp.getRANSACIterations()<< " ransacT: " << icp.getRANSACOutlierRejectionThreshold() <<" cordis: " << icp.getMaxCorrespondenceDistance() <<" tep: " << icp.getTransformationEpsilon() <<" euc: " << icp.getEuclideanFitnessEpsilon() << std::endl;   
 
  Eigen::Matrix4f T2;
  Eigen::Matrix4f Ti2 = Eigen::Matrix4f::Identity (), prev2, targetToSource2;
 // T =  best_alignment.final_transformation;//icp.getFinalTransformation();

 T2 =icp.getFinalTransformation();
  // Get the transformation from target to source
  targetToSource2 = T2.inverse();
 
std::cout << "inverse2" << targetToSource2 << std::endl;
*/
// Transform target back in source frame
  pcl::transformPointCloud (*cloud_src, *output, trans);
  final_transform = trans;

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
  pcl::PointCloud<pcl::PointXYZI>::Ptr marker (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA_filtered2 (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudC_filtered (new pcl::PointCloud<pcl::PointXYZI> );
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudD_filtered (new pcl::PointCloud<pcl::PointXYZI> );

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


setVerbosityLevel(pcl::console::L_VERBOSE); 

  // load a file
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/oldmarker.pcd", *marker) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/pcd1.pcd", *cloudA) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/pcd2.pcd", *cloudB) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/pcd3.pcd", *cloudC) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/pcd4.pcd", *cloudD) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }

  
 /*
      if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/marker.pcd", *marker) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/nm1.pcd", *cloudA) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/nm2.pcd", *cloudB) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/nm3.pcd", *cloudC) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/mike/marker/nm4.pcd", *cloudD) == -1) 
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");

      return(0);
    }
*/
// Create the filtering object
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud (cloudA);
  pass.setFilterFieldName ("intensity");
  pass.setFilterLimits (1200, 2200);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloudA_filtered);

  pass.setInputCloud (cloudB);
  pass.setFilterFieldName ("intensity");
  pass.setFilterLimits (1200, 2200);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloudB_filtered);

  pass.setInputCloud (cloudC);
  pass.setFilterFieldName ("intensity");
  pass.setFilterLimits (1100, 2200);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloudC_filtered);

  pass.setInputCloud (cloudD);
  pass.setFilterFieldName ("intensity");
  pass.setFilterLimits (1200, 2200);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloudD_filtered);


  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (cloudA_filtered);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloudA_filtered2);

///////////////////////////////////////////////


PointCloud::Ptr result1 (new PointCloud), source1 (new PointCloud), target1 (new PointCloud);
  Eigen::Matrix4f GlobalTransform1 = Eigen::Matrix4f::Identity (), pairTransform1;

  // Fill in the cloud data
  source1->width  = cloudA_filtered->width;
  source1->height = cloudA_filtered->height;
  source1->points.resize (cloudA_filtered->width * cloudA_filtered->height);
std::cout << "Done Resizing" << std::endl;
  for (size_t i = 0; i < source1->points.size (); ++i)
  {
    source1->points[i].x = cloudA_filtered->points[i].x;
    source1->points[i].y = cloudA_filtered->points[i].y;
    source1->points[i].z = cloudA_filtered->points[i].z;
  }

  // Fill in the cloud data
  target1->width  = marker->width;
  target1->height = marker->height;
  target1->points.resize (marker->width * marker->height);

  for (size_t i = 0; i < target1->points.size (); ++i)
  {
    target1->points[i].x = marker->points[i].x;
    target1->points[i].y = marker->points[i].y;
    target1->points[i].z = marker->points[i].z;
  }


    PointCloud::Ptr temp1 (new PointCloud);
    pairAlign (source1, target1, temp1, pairTransform1, false);
std::cout << "Done Aligning" << std::endl;
  pcl::transformPointCloud (*cloudA, *cloudA_aligned, pairTransform1);

////////////////////////////////////////

///////////////////////////////////////////////


PointCloud::Ptr result2 (new PointCloud), source2 (new PointCloud), target2 (new PointCloud);
  Eigen::Matrix4f GlobalTransform2 = Eigen::Matrix4f::Identity (), pairTransform2;

  // Fill in the cloud data
  source2->width  = cloudB_filtered->width;
  source2->height = cloudB_filtered->height;
  source2->points.resize (cloudB_filtered->width * cloudB_filtered->height);
std::cout << "Done Resizing" << std::endl;
  for (size_t i = 0; i < source2->points.size (); ++i)
  {
    source2->points[i].x = cloudB_filtered->points[i].x;
    source2->points[i].y = cloudB_filtered->points[i].y;
    source2->points[i].z = cloudB_filtered->points[i].z;
  }

  // Fill in the cloud data
  target2->width  = marker->width;
  target2->height = marker->height;
  target2->points.resize (marker->width * marker->height);

  for (size_t i = 0; i < target2->points.size (); ++i)
  {
    target2->points[i].x = marker->points[i].x;
    target2->points[i].y = marker->points[i].y;
    target2->points[i].z = marker->points[i].z;
  }


    PointCloud::Ptr temp2 (new PointCloud);
    pairAlign ( source2, target2, temp2, pairTransform2, false);
std::cout << "Done Aligning" << std::endl;
  pcl::transformPointCloud (*cloudB, *cloudB_aligned, pairTransform2);

////////////////////////////////////////



PointCloud::Ptr result3 (new PointCloud), source3 (new PointCloud), target3 (new PointCloud);
  Eigen::Matrix4f GlobalTransform3 = Eigen::Matrix4f::Identity (), pairTransform3;

  // Fill in the cloud data
  source3->width  = cloudC_filtered->width;
  source3->height = cloudC_filtered->height;
  source3->points.resize (cloudC_filtered->width * cloudC_filtered->height);
std::cout << "Done Resizing" << std::endl;
  for (size_t i = 0; i < source3->points.size (); ++i)
  {
    source3->points[i].x = cloudC_filtered->points[i].x;
    source3->points[i].y = cloudC_filtered->points[i].y;
    source3->points[i].z = cloudC_filtered->points[i].z;
  }

  // Fill in the cloud data
  target3->width  = marker->width;
  target3->height = marker->height;
  target3->points.resize (marker->width * marker->height);

  for (size_t i = 0; i < target3->points.size (); ++i)
  {
    target3->points[i].x = marker->points[i].x;
    target3->points[i].y = marker->points[i].y;
    target3->points[i].z = marker->points[i].z;
  }


    PointCloud::Ptr temp3 (new PointCloud);
    pairAlign (source3, target3, temp3, pairTransform3, false);
std::cout << "Done Aligning" << std::endl;
  pcl::transformPointCloud (*cloudC, *cloudC_aligned, pairTransform3);

////////////////////////////////////////



PointCloud::Ptr result4 (new PointCloud), source4 (new PointCloud), target4 (new PointCloud);
  Eigen::Matrix4f GlobalTransform4 = Eigen::Matrix4f::Identity (), pairTransform4;

  // Fill in the cloud data
  source4->width  = cloudD_filtered->width;
  source4->height = cloudD_filtered->height;
  source4->points.resize (cloudD_filtered->width * cloudD_filtered->height);
std::cout << "Done Resizing" << std::endl;
  for (size_t i = 0; i < source4->points.size (); ++i)
  {
    source4->points[i].x = cloudD_filtered->points[i].x;
    source4->points[i].y = cloudD_filtered->points[i].y;
    source4->points[i].z = cloudD_filtered->points[i].z;
  }

  // Fill in the cloud data
  target4->width  = marker->width;
  target4->height = marker->height;
  target4->points.resize (marker->width * marker->height);

  for (size_t i = 0; i < target4->points.size (); ++i)
  {
    target4->points[i].x = marker->points[i].x;
    target4->points[i].y = marker->points[i].y;
    target4->points[i].z = marker->points[i].z;
  }


    PointCloud::Ptr temp4 (new PointCloud);
    pairAlign (source4,target4, temp4, pairTransform4, false);
std::cout << "Done Aligning" << std::endl;
  pcl::transformPointCloud (*cloudD, *cloudD_aligned, pairTransform4);

////////////////////////////////////////

   sensor_msgs::PointCloud2 output1;
  pcl::toROSMsg(*cloudA_aligned, output1);
output1.header.frame_id = "base_link";
   sensor_msgs::PointCloud2 output2;
  pcl::toROSMsg(*cloudB_aligned, output2);
output2.header.frame_id = "base_link";
   sensor_msgs::PointCloud2 output3;
  pcl::toROSMsg(*cloudC_aligned, output3);
output3.header.frame_id = "base_link";
   sensor_msgs::PointCloud2 output4;
  pcl::toROSMsg(*cloudD_aligned, output4);
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
*/

    sensor_msgs::PointCloud2 output5;
 pcl::toROSMsg(*target4, output5);
output5.header.frame_id = "base_link";

// Publish the data
  pub.publish (output1);
  pub1.publish (output2);
  pub2.publish (output3);
  pub3.publish (output4);
  pub4.publish (output5);
ros::spinOnce();
std::cout << "Done" << std::endl;


}

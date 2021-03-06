#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/view_manager.h"
#include "rviz/view_controller.h"
#include "OGRE/OgreCamera.h"


#include "point_selector.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <QVariant>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

#include <pcl/filters/impl/box_clipper3D.hpp>


#include <visualization_msgs/Marker.h>

#include <pcl/filters/crop_box.h>
#include <pcl/common/angles.h>

namespace point_selector
{
markerPointsPublisher::markerPointsPublisher()
{
    updateTopic();
}

markerPointsPublisher::~markerPointsPublisher()
{
}

void markerPointsPublisher::updateTopic()
{
    nh_.param("frame_id", tf_frame_, std::string("/base_link"));
    rviz_cloud_topic_ = std::string("/selected_marker_points");
    subs_cloud_topic_ = std::string("/assembled_cloud");

    rviz_selected_pub_ = nh_.advertise<sensor_msgs::PointCloud2>( rviz_cloud_topic_.c_str(), 1 );
    pc_subs_ =  nh_.subscribe(subs_cloud_topic_.c_str(),1,&markerPointsPublisher::PointCloudsCallback, this);

    ROS_INFO_STREAM_NAMED("SelectedPointsPublisher.updateTopic", "Publishing rviz marker selected points on topic " <<  nh_.resolveName (rviz_cloud_topic_) );//<< " with frame_id " << context_->getFixedFrame().toStdString() );

    current_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    accumulated_segment_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    num_acc_points_ = 0;
    num_selected_points_ = 0;
}

void markerPointsPublisher::PointCloudsCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{
    // We only publish a message with the reception of the original pc (maybe we also do not need to copy the received pc, because is what we published!)
    if(this->accumulated_segment_pc_->points.size() == 0)
    {
        ROS_INFO_STREAM_NAMED( "SelectedPointsPublisher::PointCloudsCallback", "Received PC");
    }
    // Convert ROS PC message into a pcl point cloud
    pcl::fromROSMsg(*pc_msg, *this->current_pc_);
}

int markerPointsPublisher::processMouseEvent( rviz::ViewportMouseEvent& event )
{
    int flags = rviz::SelectionTool::processMouseEvent( event );

    // determine current selection mode
    if( event.alt() )
    {
        selecting_ = false;
    }
    else
    {
        if( event.leftDown() )
        {
            selecting_ = true;
        }
    }

    if( selecting_ )
    {
        if( event.leftUp() )
        {
            ROS_INFO_STREAM_NAMED( "SelectedPointsPublisher.processKeyEvent", "Using selected area to find a new bounding box and publish the points inside of it");
            this->_processSelectedAreaAndFindPoints();
        }
    }
    return flags;
}

int markerPointsPublisher::_processSelectedAreaAndFindPoints()
{
    rviz::SelectionManager* sel_manager = context_->getSelectionManager();
    rviz::M_Picked selection = sel_manager->getSelection();
    rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();
    int num_points = model->rowCount();
    ROS_INFO_STREAM_NAMED( "SelectedPointsPublisher._processSelectedAreaAndFindPoints", "Number of points in the selected area: " << num_points);

    // Generate a ros point cloud message with the selected points in rviz
    sensor_msgs::PointCloud2 selected_points_ros;
    selected_points_ros.header.frame_id = context_->getFixedFrame().toStdString();
    selected_points_ros.height = 1;
    selected_points_ros.width = num_points;
    selected_points_ros.point_step = 20;
    selected_points_ros.row_step = num_points * selected_points_ros.point_step;
    selected_points_ros.is_dense = false;
    selected_points_ros.is_bigendian = false;

    selected_points_ros.data.resize( selected_points_ros.row_step );
    selected_points_ros.fields.resize( 5 );
	ROS_INFO("Resized");
    selected_points_ros.fields[0].name = "x";
    selected_points_ros.fields[0].offset = 0;
    selected_points_ros.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[0].count = 1;

    selected_points_ros.fields[1].name = "y";
    selected_points_ros.fields[1].offset = 4;
    selected_points_ros.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[1].count = 1;

    selected_points_ros.fields[2].name = "z";
    selected_points_ros.fields[2].offset = 8;
    selected_points_ros.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[2].count = 1;

    selected_points_ros.fields[3].name = "intensity";
    selected_points_ros.fields[3].offset = 12;
    selected_points_ros.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[3].count = 1;   

selected_points_ros.fields[4].name = "index";
    selected_points_ros.fields[4].offset = 16;
    selected_points_ros.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[4].count = 1;
    for( int i = 0; i < num_points; i++ )
    {
        QModelIndex child_index = model->index( i, 0 );
        rviz::Property* child = model->getProp( child_index );
        rviz::VectorProperty* subchild = (rviz::VectorProperty*) child->childAt( 0 );
        rviz::FloatProperty* subchild2 = (rviz::FloatProperty*) child->childAt( 1 );
        rviz::FloatProperty* subchild3 = (rviz::FloatProperty*) child->childAt( 2 );
        Ogre::Vector3 vec = subchild->getVector();
	float vec2 = subchild2->getFloat();
	float vec3 = subchild3->getFloat();
int chil = child->numChildren();
        uint8_t* ptr = &selected_points_ros.data[0] + i * selected_points_ros.point_step;
        *(float*)ptr = vec.x;
        ptr += 4;
        *(float*)ptr = vec.y;
        ptr += 4;
        *(float*)ptr = vec.z;
        ptr += 4;
        *(float*)ptr = vec2;
        ptr += 4;
        *(float*)ptr = vec3;
        ptr += 4;
    }

    selected_points_ros.header.stamp = ros::Time::now();
    rviz_selected_pub_.publish( selected_points_ros );

    return 0;
}


} // end namespace rviz_plugin_selected_points_topic

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( point_selector::markerPointsPublisher, rviz::Tool )

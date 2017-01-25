/**
 * @file /include/test_panel/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef registration_localization_AQNODE_HPP_
#define registration_localization_AQNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/PointCloud2.h>

#include "std_msgs/String.h"
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace registration_localization {

namespace align{

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr marker_cloud;
	QNode()
	: cloud(new pcl::PointCloud<pcl::PointXYZI>())
	  , marker_cloud(new pcl::PointCloud<pcl::PointXYZI>())
	{}
	virtual ~QNode();
	bool init(std::string topic, std::string name, std::string location);
	void run();
	void align();
	void showMarker();
	void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
	std_msgs::String filename;


Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher aligned_pub;
	ros::Publisher marker_pub;
	ros::Publisher marker_aligned_pub;
	ros::Subscriber cloud_sub;
};
}
}  // namespace 

#endif /* test_panel_QNODE_HPP_ */

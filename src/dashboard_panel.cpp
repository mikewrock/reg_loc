#include "dashboard_panel.h"
#include <QDebug>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <QFileDialog>
#include <QDateTime>
#include <QTimer>



namespace registration_localization
{
	DashboardPanel::DashboardPanel( QWidget* parent )
	  : rviz::Panel( parent )
	  , save_marker()
	  , alignment_node()
	{
	  ui_.setupUi(this);

		  //SIGNAL connections
		  connect(ui_.launch_node_button, SIGNAL(clicked()), this, SLOT(onLaunchNode()));
		  connect(ui_.load_button, SIGNAL(clicked()), this, SLOT(onLoadMarker()));
		  connect(ui_.align_button, SIGNAL(clicked()), this, SLOT(onAlign()));
		  connect(ui_.show_marker, SIGNAL(clicked()), this, SLOT(onShowMarker()));


	}

	DashboardPanel::~DashboardPanel(){}

	void DashboardPanel::onLaunchNode(){

		  save_marker.init(ui_.filename_box->text().toStdString(),ui_.location_box->text().toStdString());
	}

	void DashboardPanel::onShowMarker(){

		  alignment_node.showMarker();
	}
	void DashboardPanel::onLoadMarker(){

		  
		alignment_node.init(ui_.topic_box->text().toStdString(),ui_.filename_box->text().toStdString(),ui_.location_box->text().toStdString());
	}
	void DashboardPanel::onAlign(){

		 alignment_node.align();
	}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(registration_localization::DashboardPanel,rviz::Panel)

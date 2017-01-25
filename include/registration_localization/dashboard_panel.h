#ifndef DASHBOARD_PANEL_H
#define DASHBOARD_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <ui_dashboard_panel.h>
#include <QProcess>
#include <QDir>
#include <QFileInfo>
#include "save_marker.hpp"
#include "alignment_node.hpp"

namespace registration_localization
{

class DashboardPanel: public rviz::Panel
{
Q_OBJECT
public:

  explicit DashboardPanel( QWidget* parent = 0);
  virtual ~DashboardPanel();

protected:
protected Q_SLOTS:
void onLaunchNode();
void onLoadMarker();
void onShowMarker();
void onAlign();
Q_SIGNALS:

protected:
  Ui::Form ui_;
  ros::NodeHandle nh_;

private:
	marker::QNode save_marker;
	align::QNode alignment_node;

};
}
#endif // DASHBOARD_PANEL_H

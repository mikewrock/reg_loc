#ifndef DASHBOARD_PANEL_H
#define DASHBOARD_PANEL_H

#include <ui_dashboard_panel.h>

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

namespace radbot_dashboard
{

class DashboardPanel: public rviz::Panel
{
Q_OBJECT
public:

  DashboardPanel(QWidget* parent = 0);
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

//public Q_SLOTS:

protected:
  Ui::Form ui_;
  ros::NodeHandle nh_;
  QWidget* widget_;

};

}
#endif // DASHBOARD_PANEL_H

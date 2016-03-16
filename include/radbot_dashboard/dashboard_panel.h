#ifndef DASHBOARD_PANEL_H
#define DASHBOARD_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <ui_dashboard_panel.h>
#include "thumb_widget.h"
#include <QProcess>
#include <QDir>
#include <QFileInfo>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <frontier_exploration/ExploreTaskAction.h>

namespace radbot_dashboard
{
//Qprocess Subclass to hold my arguments easily

class Process : public QProcess
{
Q_OBJECT
public:
  explicit Process(QObject * parent = 0) : QProcess(parent) {}
  QString program,path;
  QStringList args;
  virtual void start()
  {
    QProcess::start(program, QStringList()<<path<<args);
  }
};


class DashboardPanel: public rviz::Panel
{
Q_OBJECT
public:

  explicit DashboardPanel(QWidget* parent = 0);
  virtual ~DashboardPanel();
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

protected Q_SLOTS:
  void onAmclButton();
  void onSaveButton();
  void onGmappingButton();
  void onStopNavButton();
  void onExplorationButton();
  void onEstopButton();
  void onProcessError(QProcess::ProcessError error);
  void onProcessExit(int exitCode, QProcess::ExitStatus exitStatus);
  void onMapsChanged();
  void onMapSelect(int index);
  void thumbPublish();
  void thumbUpdate(float linear, float angular);
Q_SIGNALS:
  void mapsChanged();

protected:
  Ui::Form ui_;
  ros::NodeHandle nh_;
  Process* amcl_process_;
  Process* gmapping_process_;
  Process* exploration_process_;

  QDir* map_dir_;
  QFileInfoList map_file_list_;

  ros::Publisher thumb_pub_;
  ThumbWidget* tw_;
  float linear_velocity_, angular_velocity_;
  int pub_counter_;

};
}
#endif // DASHBOARD_PANEL_H

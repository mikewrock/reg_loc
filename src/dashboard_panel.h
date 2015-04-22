#ifndef DASHBOARD_PANEL_H
#define DASHBOARD_PANEL_H

#include <ui_dashboard_panel.h>
#include <QProcess>
#include <QDir>
#include <QFileInfo>

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

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
  void onProcessError(QProcess::ProcessError error);
  void onProcessExit(int exitCode, QProcess::ExitStatus exitStatus);
  void onSaveButton();
  void onMapsChanged();
  void onMapSelect(int index);
Q_SIGNALS:
  void mapsChanged();

protected:
  Ui::Form ui_;
  ros::NodeHandle nh_;

  Process* amcl_process_;
  Process* gmapping_process_;

  QDir* map_dir_;
  QFileInfoList map_file_list_;

};
}
#endif // DASHBOARD_PANEL_H

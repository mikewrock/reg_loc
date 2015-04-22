#include "dashboard_panel.h"
#include "QDebug"
#include <ros/package.h>
#include "QFileDialog"

namespace radbot_dashboard
{
DashboardPanel::DashboardPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  ui_.setupUi(this);
  setLayout(ui_.verticalLayout);
//Process setup
  amcl_process_ = new Process(this);
  gmapping_process_ = new Process(this);
  amcl_process_->program = "roslaunch";
  gmapping_process_->program = "roslaunch";
  amcl_process_->path= QString::fromStdString(ros::package::getPath("radbotlive")+"/launch/amcl.launch");
  gmapping_process_->path= QString::fromStdString(ros::package::getPath("radbotlive")+"/launch/gmapping.launch");
  amcl_process_->setProcessChannelMode(QProcess::ForwardedChannels);
  gmapping_process_->setProcessChannelMode(QProcess::ForwardedChannels);

  //Map Selection Setup
  map_dir_ = new QDir(QDir::home());
  map_dir_->setNameFilters(QStringList()<<"*.yaml");
  map_dir_->setFilter(QDir::Files);
  map_dir_->setSorting(QDir::Name);

  //SIGNAL connections
  connect(ui_.amcl_start_button, SIGNAL(clicked()), this, SLOT(onAmclButton()));
  connect(ui_.save_button, SIGNAL(clicked()), this, SLOT(onSaveButton()));
  connect(ui_.gmap_start_button, SIGNAL(clicked()), this, SLOT(onGmappingButton()));
  connect(ui_.navigation_stop_button, SIGNAL(clicked()), this, SLOT(onStopNavButton()));
  connect(amcl_process_, SIGNAL(error(QProcess::ProcessError)), this, SLOT(onProcessError(QProcess::ProcessError)));
  connect(amcl_process_, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(onProcessExit(int,QProcess::ExitStatus)));
  connect(gmapping_process_, SIGNAL(error(QProcess::ProcessError)), this, SLOT(onProcessError(QProcess::ProcessError)));
  connect(gmapping_process_, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(onProcessExit(int,QProcess::ExitStatus)));
  connect(this, SIGNAL(mapsChanged()), this, SLOT(onMapsChanged()));
  connect(ui_.map_combo, SIGNAL(activated(int)), this, SLOT(onMapSelect(int)));

  emit mapsChanged();
}

DashboardPanel::~DashboardPanel(){}

// Button Callbacks
void DashboardPanel::onAmclButton()
{
  amcl_process_->start();
  qDebug()<<"Process started, PID:"<<amcl_process_->pid();
}
void DashboardPanel::onGmappingButton()
{
  gmapping_process_->start();
  qDebug()<<"Process started, PID:"<<gmapping_process_->pid();
}
void DashboardPanel::onStopNavButton()
{
  amcl_process_->terminate();
  gmapping_process_->terminate();
}

void DashboardPanel::onSaveButton()
{
  //QString fileName = QFileDialog::getOpenFileName(this,
  //    tr("Open Map"), QDir::homePath(), tr("Map Files (*.yaml)"));
  emit mapsChanged();
}

/*
 * Map File Selection Code
 */
void DashboardPanel::onMapsChanged()
{
  QString currentText = ui_.map_combo->currentText();
  map_dir_->refresh();
  map_file_list_ = map_dir_->entryInfoList();
  ui_.map_combo->clear();
  for(int i =0; i<map_file_list_.length(); i++)
  {
    ui_.map_combo->addItem(map_file_list_.at(i).baseName());
  }
  ui_.map_combo->setCurrentIndex(ui_.map_combo->findText(currentText));
}
void DashboardPanel::onMapSelect(int index)
{
  QString arg =QString("map_file:=") + map_file_list_.at(index).absoluteFilePath();
  if(amcl_process_->args.isEmpty())
  {
    amcl_process_->args.append(arg);
  }
  else if(!amcl_process_->args.contains(arg))
  {
    int index = amcl_process_->args.indexOf(QRegExp("map_file.+"));
    if(index!=-1)
      amcl_process_->args.replace(index, arg);
    else
      amcl_process_->args.append(arg);
  }
  emit mapsChanged();
  emit configChanged();
  //qDebug()<< amcl_process_->args;
}

/*
 * Process Helpers
 */
void DashboardPanel::onProcessError(QProcess::ProcessError error)
{
  Process* tmp = dynamic_cast<Process*>(sender());
  qDebug()<<"Error, Process:"<<tmp->pid()<<" Exited with error:"<<error;
}
void DashboardPanel::onProcessExit(int exitCode, QProcess::ExitStatus exitStatus)
{
  Process* tmp = dynamic_cast<Process*>(sender());
  qDebug()<<"Process:"<<tmp->path<<" ended with code:"<<exitCode<<" and Status:"<<exitStatus;
  //qDebug()<<amcl_process_->readAllStandardOutput();
}

/*
 * Rviz Supporting Functions
 */
void DashboardPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue("Map Selection", ui_.map_combo->currentText());
}

// Load all configuration data for this panel from the given Config object.
void DashboardPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString map;
  if( config.mapGetString( "Map Selection", &map))
  {
    ui_.map_combo->setCurrentIndex(ui_.map_combo->findText(map));
  }
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(radbot_dashboard::DashboardPanel,rviz::Panel)

#include "dashboard_panel.h"


namespace radbot_dashboard
{
DashboardPanel::DashboardPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  setLayout(ui_.verticalLayout);
  //ui_.cpm_lcd->setProperty("value", QVariant(666));
}

void DashboardPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void DashboardPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}


}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(radbot_dashboard::DashboardPanel,rviz::Panel)

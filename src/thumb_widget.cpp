#include <stdio.h>
#include <math.h>

#include <QPainter>
#include <QMouseEvent>

#include "thumb_widget.h"

#include <ros/package.h>


ThumbWidget::ThumbWidget( QWidget* parent )
  : QWidget( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
  , linear_scale_( .5 )
  , angular_scale_( .785 )
  , mouseX_(75)
  , mouseY_(75)
  , thumb_pix_(new QPixmap(QString::fromStdString(ros::package::getPath("radbot_dashboard")+"/media/joystick_thumb.png")))
  , back_pix_(new QPixmap(QString::fromStdString(ros::package::getPath("radbot_dashboard")+"/media/8-direction2.png")))
{
}


void ThumbWidget::paintEvent( QPaintEvent* event )
{

  QColor background;
  QColor crosshair;
  if( isEnabled() )
  {
    background = Qt::white;
    crosshair = Qt::black;
  }
  else
  {
    background = Qt::lightGray;
    crosshair = Qt::darkGray;
  }

  //create background
  int w = width();
  int h = height();
  int size = (( w > h ) ? h : w) - 1;
  int hpad = ( w - size ) / 2;
  int vpad = ( h - size ) / 2;

  QPainter painter( this );
  painter.setBrush( background );
  painter.setPen( crosshair );

  // Draw the background square.
  painter.drawRect( QRect( hpad, vpad, size, size ));

  // Draw a cross-hair inside the square.
  //painter.drawLine( hpad, height() / 2, hpad + size, height() / 2 );
  //painter.drawLine( width() / 2, vpad, width() / 2, vpad + size );

  painter.drawPixmap(QRect( hpad, vpad, size, size ), *back_pix_);
  painter.drawPixmap(QRect(mouseX_ - size/6, mouseY_ - size/6, size/3, size/3), *thumb_pix_);

}

// Every mouse move event received here sends a velocity because Qt
// only sends us mouse move events if there was previously a
// mouse-press event while in the widget.
void ThumbWidget::mouseMoveEvent( QMouseEvent* event )
{
  sendVelocitiesFromMouse( event->x(), event->y(), width(), height() );
}

// Mouse-press events should send the velocities too, of course.
void ThumbWidget::mousePressEvent( QMouseEvent* event )
{
  sendVelocitiesFromMouse( event->x(), event->y(), width(), height() );
}

// When the mouse leaves the widget but the button is still held down,
// we don't get the leaveEvent() because the mouse is "grabbed" (by
// default from Qt).  However, when the mouse drags out of the widget
// and then other buttons are pressed (or possibly other
// window-manager things happen), we will get a leaveEvent() but not a
// mouseReleaseEvent().  Without catching this event you can have a
// robot stuck "on" without the user controlling it.
void ThumbWidget::leaveEvent( QEvent* event )
{
  stop();
}

// The ordinary way to stop: let go of the mouse button.
void ThumbWidget::mouseReleaseEvent( QMouseEvent* event )
{
  stop();
}

// Compute and emit linear and angular velocities based on Y and X
// mouse positions relative to the central square.
void ThumbWidget::sendVelocitiesFromMouse( int x, int y, int width, int height )
{
  int size = (( width > height ) ? height : width );
  int hpad = ( width - size ) / 2;
  int vpad = ( height - size ) / 2;

  mouseX_ = x;
  mouseY_ = y;

  linear_velocity_ = (1.0 - float( y - vpad ) / float( size / 2 )) * linear_scale_;
  angular_velocity_ = (1.0 - float( x - hpad ) / float( size / 2 )) * angular_scale_;
  Q_EMIT outputVelocity( linear_velocity_, angular_velocity_ );

  // update() is a QWidget function which schedules this widget to be
  // repainted the next time through the main event loop.  We need
  // this because the velocities have just changed, so the arrows need
  // to be redrawn to match.
  update();
}

// How to stop: emit velocities of 0!
void ThumbWidget::stop()
{
  linear_velocity_ = 0;
  angular_velocity_ = 0;
  mouseX_ = width()/2;
  mouseY_ = height()/2;
  Q_EMIT outputVelocity( linear_velocity_, angular_velocity_ );
  update();
}

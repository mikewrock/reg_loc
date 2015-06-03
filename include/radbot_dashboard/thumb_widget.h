#ifndef THUMB_WIDGET_H
#define THUMB_WIDGET_H

#include <QWidget>
#include <QPixmap>


class ThumbWidget: public QWidget
{
Q_OBJECT
public:
  ThumbWidget( QWidget* parent = 0 );

  // We override QWidget::paintEvent() to do custom painting.
  virtual void paintEvent( QPaintEvent* event );

  // We override the mouse events and leaveEvent() to keep track of
  // what the mouse is doing.
  virtual void mouseMoveEvent( QMouseEvent* event );
  virtual void mousePressEvent( QMouseEvent* event );
  virtual void mouseReleaseEvent( QMouseEvent* event );
  virtual void leaveEvent( QEvent* event );

  // Override sizeHint() to give the layout managers some idea of a
  // good size for this.
  virtual QSize sizeHint() const { return QSize( 175, 175 ); }

  // We emit outputVelocity() whenever it changes.
Q_SIGNALS:
  void outputVelocity( float linear, float angular );

  // mouseMoveEvent() and mousePressEvent() need the same math to
  // figure the velocities, so I put that in here.
protected:
  void sendVelocitiesFromMouse( int x, int y, int width, int height );


  void stop();

  float linear_velocity_; // In m/s
  float angular_velocity_; // In radians/s
  float linear_scale_; // In m/s
  float angular_scale_; // In radians/s
  int mouseX_;
  int mouseY_;
  QPixmap* thumb_pix_;
  QPixmap* back_pix_;
};


#endif // THUMB_WIDGET_H

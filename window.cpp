#include "2DCurveDrawer.h"
#include "window.h"

#include <QGridLayout>
#include <QLabel>
#include <QTimer>

Window::Window()
{
    setWindowTitle(tr("DrawModel"));

    CurveDrawer2D *openGL = new CurveDrawer2D(this);

    QPointF start(100,100);
    QPointF cp1(200,200);
    QPointF cp2(200,300);
    QPointF end(100,400);

    CubicBezierCurve bez(start,cp1,cp2,end);
    openGL->addCurve(bez);

    QGridLayout *layout = new QGridLayout;
    layout->addWidget(openGL, 0, 0);
    setLayout(layout);

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, openGL, &CurveDrawer2D::animate);
    timer->start(50);
}
//! [0]

#include "2DCurveDrawer.h"
#include "window.h"

#include <QGridLayout>
#include <QLabel>
#include <QTimer>

#include <stdio.h>
#include <string.h>
#include <math.h>
#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"

Window::Window()
{
    setWindowTitle(tr("DrawModel"));

    CurveDrawer2D *openGL = new CurveDrawer2D(this);

    struct NSVGimage* image;
    image = nsvgParseFromFile("/home/ferenc/Documents/camera.svg", "px", 96);

    if(image == nullptr)
        throw std::exception();

    for (NSVGshape* shape = image->shapes; shape != nullptr; shape = shape->next) {
        for (NSVGpath* path = shape->paths; path != NULL; path = path->next) {
            for (int i = 0; i < path->npts-1; i += 3) {
                float* p = &path->pts[i*2];

                QPointF start(p[0], p[1]);
                QPointF cp1(p[2], p[3]);
                QPointF cp2(p[4], p[5]);
                QPointF end(p[6], p[7]);

                CubicBezierCurve bez(start,cp1,cp2,end);
                openGL->addCurve(bez);
            }
        }
    }

    nsvgDelete(image);



    QGridLayout *layout = new QGridLayout;
    layout->addWidget(openGL, 0, 0);
    setLayout(layout);

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, openGL, &CurveDrawer2D::animate);
    timer->start(50);
}
//! [0]

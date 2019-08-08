#include "window.h"

#include <QApplication>
#include <QSurfaceFormat>
#include <QtSvg/QtSvg>
#include <gsl/gsl_multiroots.h>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    Window window;

    window.grabGesture(Qt::PanGesture);
    window.grabGesture(Qt::PinchGesture);
    window.grabGesture(Qt::SwipeGesture);


    window.show();
    return app.exec();
}

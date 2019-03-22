#include "window.h"

#include <QApplication>
#include <QSurfaceFormat>
#include <QtSvg/QtSvg>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

 //   QSurfaceFormat fmt;
 //   fmt.setSamples(4);
 //   QSurfaceFormat::setDefaultFormat(fmt);

    Window window;

    window.grabGesture(Qt::PanGesture);
    window.grabGesture(Qt::PinchGesture);
    window.grabGesture(Qt::SwipeGesture);


    window.show();
    return app.exec();
}

#ifndef CURVEDRAWER3D_H
#define CURVEDRAWER3D_H

#include <QGLViewer/qglviewer.h>
#include <QVector>
#include <QSharedPointer>
#include "cubicbeziercurve.h"

class CurveDrawer3D : public QGLViewer
{
private:
    QVector<CubicBezierCurve> curves;
    float color[3];

public:
    void addCurve(const CubicBezierCurve&);
    virtual void init();
    virtual void draw();

};

#endif // CURVEDRAWER3D_H

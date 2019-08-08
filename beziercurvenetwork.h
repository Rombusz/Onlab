#ifndef BEZIERCURVENETWORK_H
#define BEZIERCURVENETWORK_H

#include "cubicbeziercurve.h"
#include <map>

class VectorComparator{
public:
    bool operator()(const QVector3D& a, const QVector3D& b) const {

        return a.length() < b.length();

    }

};

class BezierCurveNetwork
{
    QVector<CubicBezierCurve> curves;

public:
    void addCurve(const CubicBezierCurve& curve);
    QVector<CubicBezierCurve> getSegmentsAtPoint(const QVector3D& point) const;
    const QVector<CubicBezierCurve>& getCurves() const;
    QVector<CubicBezierCurve> getCurvesCopy() const;
    void ReplaceSegmentsAtPoint(const QVector3D& point, const QVector<CubicBezierCurve>& curves);

};

#endif // BEZIERCURVENETWORK_H

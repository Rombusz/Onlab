#include "beziercurvenetwork.h"

void BezierCurveNetwork::addCurve(const CubicBezierCurve& curve){

    this->curves.push_back(curve);

}

QVector<CubicBezierCurve> BezierCurveNetwork::getSegmentsAtPoint(const QVector3D& point) const {

    QVector<CubicBezierCurve> vec;

    for(auto curve : this->curves){

        if((curve.getEndPoint() - point).length() <= 0.1f || (curve.getStartPoint() - point).length() <= 0.1f )vec.push_back(curve);

    }

    return vec;

}

QVector<CubicBezierCurve>& BezierCurveNetwork::getCurves(){

    return this->curves;

}

QVector<CubicBezierCurve> BezierCurveNetwork::getCurvesCopy() const {

    return this->curves;

}

void BezierCurveNetwork::ReplaceSegmentsAtPoint(const QVector3D& point, const QVector<CubicBezierCurve>& curvesToAdd){

    this->curves.erase(std::remove_if(curves.begin(),curves.end(), [&](CubicBezierCurve curve){return (curve.getEndPoint() == point || curve.getStartPoint() == point);} ) ,curves.end());
    this->curves.append(curvesToAdd);

}

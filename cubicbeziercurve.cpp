#include "cubicbeziercurve.h"
#include <iostream>

CubicBezierCurve::CubicBezierCurve()
: startPoint(), controlPoint1(), controlPoint2(),endPoint()
{

}

CubicBezierCurve::CubicBezierCurve(const QPointF& startPoint, const QPointF& endPoint)
: startPoint(startPoint), controlPoint1(), controlPoint2(),endPoint(endPoint)
{

}

CubicBezierCurve::CubicBezierCurve(const QPointF& startPoint,const QPointF& controlPoint1,const QPointF& controlPoint2, const QPointF& endPoint)
: startPoint(startPoint), controlPoint1(controlPoint1), controlPoint2(controlPoint2),endPoint(endPoint)
{

}


QPointF CubicBezierCurve::calculatePoint(const qfloat16& t) const{

    qfloat16 b30 = pow(1-t,3);
    qfloat16 b31 = 3*pow(1-t,2)*t;
    qfloat16 b32 = 3*(1-t)*pow(t,2);
    qfloat16 b33 = pow(t,3);

    std::cout << "B30 " << b30 << " B31 " << b31 << " B32 " << b32 << " B33 " << b33 << std::endl;

    return b30*startPoint+b31*controlPoint1+b32*controlPoint2+b33*endPoint;

}

void CubicBezierCurve::setStartPoint(const QPointF& startPoint){

    this->startPoint = startPoint;

}

void CubicBezierCurve::setEndPoint(const QPointF& endPoint){

    this->endPoint = endPoint;

}

void CubicBezierCurve::setControlPoint1(const QPointF& controlPoint){

    this->controlPoint1 = controlPoint;

}
void CubicBezierCurve::setControlPoint2(const QPointF& controlPoint){

    this->controlPoint2 = controlPoint;

}

void CubicBezierCurve::draw(QPainter& painter, const QPaintEvent& event, const QPen& pen,const QPen& cpolypen, const QBrush& background, int numberOfSamples, bool showControlPolygon) const{


    qfloat16 increment = 1.0f/(numberOfSamples-1);

    painter.save();

    painter.setPen(cpolypen);
    painter.setBrush(background);

    qfloat16 parameter = increment;

    if(showControlPolygon){

        painter.drawLine(startPoint,controlPoint1);
        painter.drawLine(controlPoint1,controlPoint2);
        painter.drawLine(controlPoint2,endPoint);

        painter.drawEllipse(controlPoint1,4.0,4.0);
        painter.drawEllipse(controlPoint2,4.0,4.0);

    }
    painter.drawEllipse(startPoint,4.0,4.0);
    painter.drawEllipse(endPoint,4.0,4.0);

    painter.setPen(pen);

    for(;parameter<1.0f ;parameter+=increment ){

        std::cout << parameter << std::endl;

        QPointF segmentStart = this->calculatePoint(parameter-increment);
        QPointF segmentEnd = this->calculatePoint(parameter);

        painter.drawLine(segmentStart,segmentEnd);

    }

    QPointF segmentStart = this->calculatePoint(parameter-increment);
    QPointF segmentEnd = this->calculatePoint(1.0f);

    painter.drawLine(segmentStart,segmentEnd);

    painter.restore();

};

#include "cubicbeziercurve.h"
#include <iostream>

CubicBezierCurve::CubicBezierCurve()
: startPoint(), controlPoint1(), controlPoint2(),endPoint(), offset(0,0,0),scale(1)
{

}

CubicBezierCurve::CubicBezierCurve(const QVector3D& startPoint, const QVector3D& endPoint)
: startPoint(startPoint), controlPoint1(), controlPoint2(),endPoint(endPoint), offset(0,0,0),scale(1)
{

}

CubicBezierCurve::CubicBezierCurve(const QVector3D& startPoint,const QVector3D& controlPoint1,const QVector3D& controlPoint2, const QVector3D& endPoint)
: startPoint(startPoint), controlPoint1(controlPoint1), controlPoint2(controlPoint2),endPoint(endPoint), offset(0,0,0),scale(1)
{

}

QVector3D CubicBezierCurve::calculatePoint(const qfloat16& t) const{

    qfloat16 b30 = pow(1-t,3);
    qfloat16 b31 = 3*pow(1-t,2)*t;
    qfloat16 b32 = 3*(1-t)*pow(t,2);
    qfloat16 b33 = pow(t,3);

    return b30*startPoint+b31*controlPoint1+b32*controlPoint2+b33*endPoint;

}

void CubicBezierCurve::setStartPoint(const QVector3D& startPoint){

    this->startPoint = startPoint;

}

void CubicBezierCurve::setEndPoint(const QVector3D& endPoint){

    this->endPoint = endPoint;

}

void CubicBezierCurve::setControlPoint1(const QVector3D& controlPoint){

    this->controlPoint1 = controlPoint;

}
void CubicBezierCurve::setControlPoint2(const QVector3D& controlPoint){

    this->controlPoint2 = controlPoint;

}

void CubicBezierCurve::draw(QPainter& painter, const QPaintEvent& event, const QPen& pen,const QPen& cpolypen, const QBrush& background, int numberOfSamples, bool showControlPolygon) const{


    qfloat16 increment = 1.0f/(numberOfSamples-1);

    painter.save();

    painter.setPen(cpolypen);
    painter.setBrush(background);

    QTransform transform;

    transform.scale(this->scale, this->scale);
    transform.translate( this->offset.x(), this->offset.y() );

    painter.setTransform(transform);

    qfloat16 parameter = increment;

    if(showControlPolygon){

        painter.drawLine(startPoint.toPointF(),controlPoint1.toPointF());
        painter.drawLine(controlPoint1.toPointF(),controlPoint2.toPointF());
        painter.drawLine(controlPoint2.toPointF(),endPoint.toPointF());

        painter.drawEllipse(controlPoint1.toPointF(),4.0,4.0);
        painter.drawEllipse(controlPoint2.toPointF(),4.0,4.0);

    }
    painter.drawEllipse(startPoint.toPointF(),4.0,4.0);
    painter.drawEllipse(endPoint.toPointF(),4.0,4.0);

    painter.setPen(pen);

    for(;parameter<1.0f ;parameter+=increment ){

        QVector3D segmentStart = this->calculatePoint(parameter-increment);
        QVector3D segmentEnd = this->calculatePoint(parameter);

        painter.drawLine(segmentStart.toPointF(),segmentEnd.toPointF());

    }

    QVector3D segmentStart = this->calculatePoint(parameter-increment);
    QVector3D segmentEnd = this->calculatePoint(1.0f);

    painter.drawLine(segmentStart.toPointF(),segmentEnd.toPointF());

    painter.restore();

};

void CubicBezierCurve::setTranslate(const QVector2D& offsetVector){

    this->offset = offsetVector;

}
void CubicBezierCurve::setScale(const qfloat16& uniformscale){

    this->scale = uniformscale;

}

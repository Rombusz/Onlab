#ifndef CUBICBEZIERCURVE_H
#define CUBICBEZIERCURVE_H

#include <qpoint.h>
#include <QVector>
#include <QPainter>
#include <QPaintEvent>
#include <QFloat16>
#include <QtMath>

class CubicBezierCurve final
{

    private:
        QPointF startPoint;
        QPointF controlPoint1;
        QPointF controlPoint2;
        QPointF endPoint;

    public:
        CubicBezierCurve();
        CubicBezierCurve(const QPointF& startPoint, const QPointF& endPoint);
        CubicBezierCurve(const QPointF& startPoint,const QPointF& controlPoint1,const QPointF& controlPoint2, const QPointF& endPoint);

        void setStartPoint(const QPointF& startPoint);
        void setEndPoint(const QPointF& endPoint);
        void setControlPoint1(const QPointF& controlPoint);
        void setControlPoint2(const QPointF& controlPoint);

        void draw(QPainter& painter, const QPaintEvent& event, const QPen& pen,const QPen& cpolypen, const QBrush& background, int numberOfSamples) const;
        QPointF calculatePoint(const qfloat16& param) const;
        virtual ~CubicBezierCurve(){}

};

#endif

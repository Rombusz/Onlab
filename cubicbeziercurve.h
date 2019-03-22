#ifndef CUBICBEZIERCURVE_H
#define CUBICBEZIERCURVE_H

#include <qpoint.h>
#include <QVector>
#include <QPainter>
#include <QPaintEvent>
#include <QFloat16>
#include <QtMath>
#include <QMatrix4x4>

class CubicBezierCurve final
{

    private:
        QPointF startPoint;
        QPointF controlPoint1;
        QPointF controlPoint2;
        QPointF endPoint;
        QVector2D offset;
        qfloat16 scale;

    public:
        CubicBezierCurve();
        CubicBezierCurve(const QPointF& startPoint, const QPointF& endPoint);
        CubicBezierCurve(const QPointF& startPoint,const QPointF& controlPoint1,const QPointF& controlPoint2, const QPointF& endPoint);

        void setStartPoint(const QPointF& startPoint);
        void setEndPoint(const QPointF& endPoint);
        void setControlPoint1(const QPointF& controlPoint);
        void setControlPoint2(const QPointF& controlPoint);

        void draw(QPainter& painter, const QPaintEvent& event, const QPen& pen,const QPen& cpolypen, const QBrush& background, int numberOfSamples, bool showControlPolygon) const;
        QPointF calculatePoint(const qfloat16& param) const;
        void setTranslate(const QVector2D& offsetVector);
        void setScale(const qfloat16& uniformscale);
        virtual ~CubicBezierCurve(){}

};

#endif

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
        QVector3D startPoint;
        QVector3D controlPoint1;
        QVector3D controlPoint2;
        QVector3D endPoint;
        QVector3D offset;
        qfloat16 scale;
        qfloat16 minX, minY, maxX, maxY;

    public:
        CubicBezierCurve();
        CubicBezierCurve(const QVector3D& startPoint, const QVector3D& endPoint);
        CubicBezierCurve(const QVector3D& startPoint,const QVector3D& controlPoint1,const QVector3D& controlPoint2, const QVector3D& endPoint);

        void setStartPoint(const QVector3D& startPoint);
        void setEndPoint(const QVector3D& endPoint);
        void setControlPoint1(const QVector3D& controlPoint);
        void setControlPoint2(const QVector3D& controlPoint);

        void draw(QPainter& painter, const QPaintEvent& event, const QPen& pen,const QPen& cpolypen, const QBrush& background, int numberOfSamples, bool showControlPolygon) const;
        QVector3D calculatePoint(const qfloat16& param) const;
        void setTranslate(const QVector2D& offsetVector);
        void setScale(const qfloat16& uniformscale);
        virtual ~CubicBezierCurve(){}

};

#endif

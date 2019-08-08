/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "CurveDrawer2D.h"
#include "helper.h"

#include <QPainter>
#include <QTimer>
#include <iostream>

CurveDrawer2D::CurveDrawer2D(QWidget *parent)
    :QOpenGLWidget(parent),curves(),isControlPolygonVisible(false),offset(0,0),scaleFactor(1),selectedSegmentEndpoint(0,0),isEndpointSelected(false)
{
    setAutoFillBackground(false);
}

void CurveDrawer2D::animate()
{

}

void CurveDrawer2D::paintEvent(QPaintEvent *event)
{
    QPainter painter;
    painter.begin(this);
    painter.setRenderHint(QPainter::Antialiasing);

    QPen linepen(Qt::green,3.0,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
    QPen cpolygonpen(Qt::gray);
    QBrush brush(Qt::white);

    painter.fillRect(event->rect(),brush);

    for(const CubicBezierCurve& curve : this->curves){

        curve.draw(painter, *event, linepen, cpolygonpen, brush, 15, isControlPolygonVisible);

    }

    QBrush redBrush(Qt::red);
    QPen redPen(Qt::red);

    painter.setBrush(redBrush);
    painter.setPen(redPen);


    if(isEndpointSelected){

        QTransform transform;

        transform = transform.scale(scaleFactor, scaleFactor) * transform.translate(offset.x(), offset.y());

        QVector3D segmentPoint(selectedSegmentEndpoint.x(), selectedSegmentEndpoint.y(), 1);

        segmentPoint = transform*segmentPoint;

        painter.drawEllipse(segmentPoint.toPointF(),4.0f,4.0f);

    }
    painter.end();
}

void CurveDrawer2D::showControlPolygon(){

    this->isControlPolygonVisible = true;

}
void CurveDrawer2D::hideControlPolygon(){

    this->isControlPolygonVisible = false;

}

void CurveDrawer2D::mouseMoveEvent(QMouseEvent *event){

    if(event->buttons() & Qt::LeftButton){

        QPoint currentPos = event->pos()/this->scaleFactor;
        QPoint offsetPoint = currentPos - prevClickPos;
        QVector2D offsetVector(offsetPoint.x(),offsetPoint.y());

        this->offset += offsetVector;

        for(auto& curve : this->curves){

            curve.setTranslate((this->offset));

        }

        prevClickPos = event->pos()/this->scaleFactor;
        event->accept();
        this->update();

    }

}

void CurveDrawer2D::wheelEvent(QWheelEvent *event){

    std::cout << event->delta() << std::endl;

    qfloat16 scaleChange = event->delta()/1000.0f;
    this->scaleFactor += scaleChange;

    for(auto& curve : this->curves){

        curve.setScale(this->scaleFactor);

    }

    event->accept();
    this->update();

}

QVector2D CurveDrawer2D::getSelectedPoint() const {

    return this->selectedSegmentEndpoint;

}

bool CurveDrawer2D::isPointSelected() const{

    return this->isEndpointSelected;

}


void CurveDrawer2D::mousePressEvent(QMouseEvent *event){

    if(event->buttons() & Qt::LeftButton){

        isClicked = true;
        prevClickPos = event->pos()/this->scaleFactor;

    }

    if(event->buttons() & Qt::RightButton){

        QVector3D selectedPoint(event->x(),event->y(), 1);
        QVector3D closestPoint = curves.last().getEndPoint();
        QTransform transform;

        transform = transform.scale(1/scaleFactor, 1/scaleFactor) * transform.translate(-offset.x(), -offset.y());
        std::cout << "Scale and offset: " << scaleFactor << " " << offset.x() << " " << offset.y() << std::endl;
        selectedPoint=transform*selectedPoint;

        float closestDistance = (closestPoint - selectedPoint).length();

        for(const auto& curve : curves){

            QVector3D endPoint(curve.getEndPoint().x(), curve.getEndPoint().y(), 1);
            QVector3D startPoint(curve.getStartPoint().x(), curve.getStartPoint().y(), 1);

            float endPointdistance = (endPoint-selectedPoint).length();
            float startPointdistance = (startPoint-selectedPoint).length();

            if( endPointdistance <= startPointdistance && endPointdistance < closestDistance ){
                closestPoint = endPoint;
                closestDistance = endPointdistance;
            }
            else if(startPointdistance < endPointdistance && startPointdistance < closestDistance ){
                closestPoint = startPoint;
                closestDistance = startPointdistance;
            }
        }

        std::cout << "Curve point in transformed space:" << closestPoint.x() << " " << closestPoint.y() << std::endl;

        if(closestDistance < 4.0f){

            this->selectedSegmentEndpoint = QVector2D(closestPoint.x(), closestPoint.y());
            isEndpointSelected = true;

        }


        event->accept();
        this->update();

    }

}

void CurveDrawer2D::addNetwork(const BezierCurveNetwork& network){

    this->curves.append(network.getCurvesCopy());

}

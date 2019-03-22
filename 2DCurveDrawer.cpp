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

#include "2DCurveDrawer.h"
#include "helper.h"

#include <QPainter>
#include <QTimer>
#include <iostream>

static void gestureEventHandler(QGestureEvent* event){

    if (QGesture *swipe = event->gesture(Qt::SwipeGesture))
        std::cout << "Swipe gesture" << std::endl;
    else if (QGesture *pan = event->gesture(Qt::PanGesture))
        std::cout << "Pan gesture" << std::endl;
    if (QGesture *pinch = event->gesture(Qt::PinchGesture))
        std::cout << "Pinch gesture" << std::endl;

}

CurveDrawer2D::CurveDrawer2D(QWidget *parent)
    :QOpenGLWidget(parent),curves(),isControlPolygonVisible(false),offset(0,0)
{
    setMinimumSize(500,500);
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

    for(CubicBezierCurve& curve : this->curves){

        curve.draw(painter, *event, linepen, cpolygonpen, brush, 15, isControlPolygonVisible);

    }

    painter.end();
}

void CurveDrawer2D::addCurve(const CubicBezierCurve& curve){

    this->curves.push_back(curve);

}

void CurveDrawer2D::showControlPolygon(){

    this->isControlPolygonVisible = true;

}
void CurveDrawer2D::hideControlPolygon(){

    this->isControlPolygonVisible = false;

}

void CurveDrawer2D::mouseMoveEvent(QMouseEvent *event){

    if(event->buttons() & Qt::LeftButton){

        QPoint currentPos = event->pos();
        QPoint offsetPoint = currentPos - prevClickPos;
        QVector2D offsetVector(offsetPoint.x(),offsetPoint.y());

        this->offset += offsetVector;

        for(auto& curve : this->curves){

            curve.setTranslate(this->offset);

        }

        std::cout << this->offset.x() << " " << this->offset.y() << std::endl;

        prevClickPos = event->pos();
        event->accept();
        this->update();

    }

}

void CurveDrawer2D::mousePressEvent(QMouseEvent *event){

    if(event->buttons() & Qt::LeftButton){

        isClicked = true;
        prevClickPos = event->pos();

    }

}

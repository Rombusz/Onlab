#include "window.h"

#include <stdio.h>
#include <iostream>
#include <string.h>
#include <math.h>
#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"

Window::Window()
    :curveDrawer2D(this),
      curveDrawer3D(),
      openFileButton("Open SVG",this),
      checkBox(),
      mainLayout(QBoxLayout::Down),
      toolLayout(QBoxLayout::LeftToRight),
      canvasLayout(QBoxLayout::LeftToRight),
      checkBoxLabel("Show Control Polygon")
{
    setWindowTitle(tr("DrawModel"));
    openFileButton.connect(&openFileButton,SIGNAL (clicked()), this, SLOT (onFileButtonClick()));

    canvasLayout.addWidget(&curveDrawer2D);
    canvasLayout.addWidget(&curveDrawer3D);
    toolLayout.addWidget(&openFileButton);
    toolLayout.addWidget(&checkBoxLabel);
    toolLayout.addWidget(&checkBox);
    mainLayout.addLayout(&canvasLayout);
    mainLayout.addLayout(&toolLayout);
    setLayout(&mainLayout);
    setAttribute(Qt::WA_AcceptTouchEvents);
    setMouseTracking(true);


    connect(&checkBox, SIGNAL(clicked(bool)),this, SLOT(onCheckBoxStateChange(bool)));

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, &curveDrawer2D, &CurveDrawer2D::animate);
    timer->start(50);
}

void Window::onFileButtonClick(){

    QString fileName = QFileDialog::getOpenFileName(this, tr("Open SVG"), "/home/", tr("SVG Files (*.svg)"));

    struct NSVGimage* image;
    image = nsvgParseFromFile(fileName.toUtf8().data(), "px", 96);

    if(image == nullptr)
        throw std::exception();

    for (NSVGshape* shape = image->shapes; shape != nullptr; shape = shape->next) {
        for (NSVGpath* path = shape->paths; path != NULL; path = path->next) {
            for (int i = 0; i < path->npts-1; i += 3) {
                float* p = &path->pts[i*2];

                QVector3D start(p[0], p[1], 0);
                QVector3D cp1(p[2], p[3], 0);
                QVector3D cp2(p[4], p[5], 0);
                QVector3D end(p[6], p[7], 0);

                CubicBezierCurve bez(start,cp1,cp2,end);
                curveDrawer2D.addCurve(bez);
                curveDrawer3D.addCurve(bez);
            }
        }
    }

    nsvgDelete(image);

    this->curveDrawer2D.update();

}

void Window::onCheckBoxStateChange(bool checkBoxState){

    if(checkBoxState)this->curveDrawer2D.showControlPolygon();
    else this->curveDrawer2D.hideControlPolygon();

    this->curveDrawer2D.update();

}


//! [0]

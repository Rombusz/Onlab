#include "window.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"

Window::Window()
    :curveDrawer(this),
      openFileButton("Open SVG",this),
      checkBox(),
      mainLayout(QBoxLayout::Down),
      toolLayout(QBoxLayout::LeftToRight),
      checkBoxLabel("Show Control Polygon")
{
    setWindowTitle(tr("DrawModel"));
    openFileButton.connect(&openFileButton,SIGNAL (clicked()), this, SLOT (onFileButtonClick()));

    mainLayout.addWidget(&curveDrawer);
    toolLayout.addWidget(&openFileButton);
    toolLayout.addWidget(&checkBoxLabel);
    toolLayout.addWidget(&checkBox);
    mainLayout.addLayout(&toolLayout);
    setLayout(&mainLayout);

    connect(&checkBox, SIGNAL(clicked(bool)),this, SLOT(onCheckBoxStateChange(bool)));

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, &curveDrawer, &CurveDrawer2D::animate);
    timer->start(50);
}

void Window::onFileButtonClick(){

    QString fileName = QFileDialog::getOpenFileName(this, tr("Open SVG"), "/home/", tr("SVG Files (*.svg)"));

    struct NSVGimage* image;
    image = nsvgParseFromFile("/home/ferenc/Documents/camera.svg", "px", 96);

    if(image == nullptr)
        throw std::exception();

    for (NSVGshape* shape = image->shapes; shape != nullptr; shape = shape->next) {
        for (NSVGpath* path = shape->paths; path != NULL; path = path->next) {
            for (int i = 0; i < path->npts-1; i += 3) {
                float* p = &path->pts[i*2];

                QPointF start(p[0], p[1]);
                QPointF cp1(p[2], p[3]);
                QPointF cp2(p[4], p[5]);
                QPointF end(p[6], p[7]);

                CubicBezierCurve bez(start,cp1,cp2,end);
                curveDrawer.addCurve(bez);
            }
        }
    }

    nsvgDelete(image);

    this->curveDrawer.update();

}

void Window::onCheckBoxStateChange(bool checkBoxState){

    if(checkBoxState)this->curveDrawer.showControlPolygon();
    else this->curveDrawer.hideControlPolygon();

    this->curveDrawer.update();

}

//! [0]

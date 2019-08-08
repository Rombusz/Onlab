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
      checkBoxLabel("Show Control Polygon"),
      sliderLambda(Qt::Horizontal),
      sliderZ(Qt::Horizontal),
      solver(),
      sliderLambdaLabel("Lambda"),
      sliderZLabel("Z")
{
    setWindowTitle(tr("DrawModel"));
    openFileButton.connect(&openFileButton,SIGNAL (clicked()), this, SLOT (onFileButtonClick()));

    sliderZ.setRange(0,200);
    sliderZ.setTickPosition(QSlider::TicksBelow);
    sliderLambda.setRange(0,1000);
    sliderLambda.setTickPosition(QSlider::TicksBelow);

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
    connect(&sliderZ, SIGNAL(valueChanged(int)), this, SLOT(updateLabelZLabel(int)));
    connect(&sliderLambda, SIGNAL(valueChanged(int)), this, SLOT(updateLabelLambda(int)));

    timer->start(50);
}

void Window::updateLabelLambda(int number){

    this->sliderLambdaLabel.setNum(number/(float)(sliderLambda.maximum() - sliderLambda.minimum() ));

}

void Window::updateLabelZLabel(int number){

    this->sliderZLabel.setNum(number);

}

void Window::onFileButtonClick(){

    QString fileName = QFileDialog::getOpenFileName(this, tr("Open SVG"), "/home/", tr("SVG Files (*.svg)"));

    struct NSVGimage* image;
    image = nsvgParseFromFile(fileName.toUtf8().data(), "px", 96);

    if(image == nullptr)
        throw std::exception();

    BezierCurveNetwork bezNetwork;

    for (NSVGshape* shape = image->shapes; shape != nullptr; shape = shape->next) {
        for (NSVGpath* path = shape->paths; path != NULL; path = path->next) {
            for (int i = 0; i < path->npts-1; i += 3) {
                float* p = &path->pts[i*2];

                QVector3D start(p[0], p[1], 0);
                QVector3D cp1(p[2], p[3], 0);
                QVector3D cp2(p[4], p[5], 0);
                QVector3D end(p[6], p[7], 0);

                CubicBezierCurve bez(start,cp1,cp2,end);
                bezNetwork.addCurve(bez);
            }
        }
    }

    nsvgDelete(image);

    curveDrawer2D.addNetwork(bezNetwork);
    //curveDrawer3D.addCurves(bezNetwork.getCurves());
    solver.setNetwork(bezNetwork);

    this->curveDrawer2D.update();
    this->curveDrawer3D.update();

}

void Window::onCheckBoxStateChange(bool checkBoxState){

    if(checkBoxState)this->curveDrawer2D.showControlPolygon();
    else this->curveDrawer2D.hideControlPolygon();

    this->curveDrawer2D.update();

}

void Window::keyReleaseEvent(QKeyEvent *event){

    if(event->key() & Qt::Key_Enter && this->curveDrawer2D.isPointSelected()){

        QVector3D startPoint(curveDrawer2D.getSelectedPoint().x(), curveDrawer2D.getSelectedPoint().y() ,0 );

        std::vector<float> initParams;
        initParams.push_back(sliderZ.value());
        initParams.push_back(sliderLambda.value());
        solver.setInitialGuess(initParams);
        solver.CreateBaselineReconstruction(startPoint);
        this->curveDrawer3D.addCurves(solver.getCurrentSolution().getCurves());
        this->curveDrawer3D.update();

    }

    event->accept();

}


//! [0]

#include "curvedrawer3d.h"

void CurveDrawer3D::addCurve(const CubicBezierCurve& curve){

    this->curves.push_back(curve);
    color[0] = 1;
    color[1] = 1;
    color[2] = 1;

}

void CurveDrawer3D::init(){

    glMaterialfv(GL_FRONT, GL_AMBIENT, color);

}

void CurveDrawer3D::draw(){

    glBegin(GL_LINE_STRIP);
    glPolygonMode(GL_FRONT, GL_LINE);
    glDisable(GL_LIGHTING);
    glColor3f(1, 1, 1);
    glNormal3f(0,0,1);

    for(auto& curve : this->curves){

        int sampleNumber = 10;
        float stepSize = 1.0f/sampleNumber;

        for (int i = 0; i <= sampleNumber ;i++) {

            QVector3D vec = curve.calculatePoint(i*stepSize)/1200.0f;

            glVertex3f(vec.x(), vec.y(), vec.z());

        }

    }

    glEnd();

}

#include "curvedrawer3d.h"

void CurveDrawer3D::addCurve(const CubicBezierCurve& curve){

    this->curves.push_back(curve);

}

void CurveDrawer3D::addCurves(const QVector<CubicBezierCurve>& curves){

    this->curves.append(curves);

}

void CurveDrawer3D::init(){

    color[0] = 1;
    color[1] = 1;
    color[2] = 1;
    glMaterialfv(GL_FRONT, GL_AMBIENT, color);

}

void CurveDrawer3D::draw(){

    for(auto& curve : this->curves){

        int sampleNumber = 10;
        float stepSize = 1.0f/sampleNumber;

        glBegin(GL_LINE_STRIP);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_LIGHTING);
        glColor3f(1, 1, 1);
        glNormal3f(-this->camera()->viewDirection().x,-this->camera()->viewDirection().y,-this->camera()->viewDirection().z);


        for (int i = 0; i <= sampleNumber ;i++) {

            QVector3D vec = curve.calculatePoint(i*stepSize)/1200.0f;

            glVertex3f(vec.x(), vec.y(), vec.z());

        }

        glEnd();

    }



}

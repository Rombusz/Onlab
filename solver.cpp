#include "solver.h"
#include "iostream"

Solver::Solver(){



}

void Solver::setNetwork(const BezierCurveNetwork& net){

    this->network = net;

}

int Solver::BaselineReconstructionFunction(const gsl_vector * x, void *params, gsl_vector * f){

    SolverParams* param = reinterpret_cast<SolverParams*>(params);
    QVector<CubicBezierCurve> BezierSegments3D;
    QVector<CubicBezierCurve>* segments = param->segments;

  /*  for(int i=0; i < segments->size()*4*3; i+=12){

        QVector3D start( gsl_vector_get (x, i), gsl_vector_get (x, i+1), gsl_vector_get (x, i+2));
        QVector3D ControlPoint1(gsl_vector_get (x, i+3), gsl_vector_get (x, i+4), gsl_vector_get (x, i+5));
        QVector3D ControlPoint2(gsl_vector_get (x, i+6), gsl_vector_get (x, i+7), gsl_vector_get (x, i+8));
        QVector3D end(gsl_vector_get (x, i+9), gsl_vector_get (x, i+10), gsl_vector_get (x, i+11));

        BezierSegments3D.push_back(CubicBezierCurve(start,ControlPoint1, ControlPoint2, end));

    }*/

    for(int i=0; i < segments->size(); i++){

        QVector3D start,end;

        if(param->selectedBaselinePoint == segments->at(i).getStartPoint()){
            start = QVector3D( segments->at(i).getStartPoint().x(), segments->at(i).getStartPoint().y(), gsl_vector_get (x, 0));
            end = QVector3D(segments->at(i).getEndPoint().x(), segments->at(i).getEndPoint().y(), segments->at(i).getEndPoint().z());
        }
        else{
            start = QVector3D( segments->at(i).getStartPoint().x(), segments->at(i).getStartPoint().y(), segments->at(i).getStartPoint().z());
            end = QVector3D(segments->at(i).getEndPoint().x(), segments->at(i).getEndPoint().y(), gsl_vector_get (x, 0));
        }
            QVector3D ControlPoint1(segments->at(i).getControlPoint1().x(), segments->at(i).getControlPoint1().y(), segments->at(i).getControlPoint1().z());
            QVector3D ControlPoint2(segments->at(i).getControlPoint2().x(), segments->at(i).getControlPoint2().y(), segments->at(i).getControlPoint2().z());

            BezierSegments3D.push_back(CubicBezierCurve(start,ControlPoint1, ControlPoint2, end));

        }

    float CorthoLambda = gsl_vector_get (x, x->size-1);

    std::vector<float> gradAcc = EaccuracyGrad(*segments,BezierSegments3D);
    std::vector<float> gradVar = EvariationGrad(*segments,BezierSegments3D);
    std::vector<float> gradFore = EforeshorteningGrad(*segments,BezierSegments3D);
    std::vector<float> gradCortho = CorthoGrad(BezierSegments3D);
/*
    for(int i=0;i< gradAcc.size();i++){

        float result = 10*gradAcc.at(i) + 2*gradVar.at(i) + 0.001*gradFore.at(i) + CorthoLambda*gradCortho.at(i);
        gsl_vector_set(f,i, result);

    }*/

    if(param->selectedBaselinePoint == segments->at(0).getStartPoint()){
        gsl_vector_set(f,0, 1 + CorthoLambda*gradCortho.at(2));
        gsl_vector_set(f,0, 1 + gradCortho.at(2));
    }
    else{
        gsl_vector_set(f,1, 1 + CorthoLambda*gradCortho.at(11));
        gsl_vector_set(f,1, gradCortho.at(11));
    }

    float c = Cortho(BezierSegments3D);

    gsl_vector_set(f,f->size-1, c);

    return GSL_SUCCESS;

}

std::vector<float> Solver::CorthoGrad(const QVector<CubicBezierCurve>& curves3D){

    std::vector<float> gradient;

    for (int i=0;i<curves3D.size();i++) {

        QVector3D gradientVectorStartPoint(0,0,0);
        QVector3D gradientVectorCp1(0,0,0);
        QVector3D gradientVectorCp2(0,0,0);
        QVector3D gradientVectorEndPoint(0,0,0);

        for (int j=0;j<curves3D.size();j++) {

            if(i!=j){

                if(curves3D.at(i).getStartPoint() == curves3D.at(j).getStartPoint()){

                     gradientVectorStartPoint -= (curves3D.at(j).getControlPoint1() - curves3D.at(j).getStartPoint());
                     gradientVectorCp1 += (curves3D.at(j).getControlPoint1() - curves3D.at(j).getStartPoint());

                }

                if(curves3D.at(i).getStartPoint() == curves3D.at(j).getEndPoint()){

                    gradientVectorStartPoint -= (curves3D.at(j).getControlPoint2() - curves3D.at(j).getEndPoint());
                    gradientVectorCp1 += (curves3D.at(j).getControlPoint2() - curves3D.at(j).getEndPoint());

                }

                if(curves3D.at(i).getEndPoint() == curves3D.at(j).getStartPoint()){

                    gradientVectorEndPoint -= (curves3D.at(j).getControlPoint1() - curves3D.at(j).getStartPoint());
                    gradientVectorCp2 += (curves3D.at(j).getControlPoint1() - curves3D.at(j).getStartPoint());

                }

                if(curves3D.at(i).getEndPoint() == curves3D.at(j).getEndPoint()){

                    gradientVectorEndPoint -= (curves3D.at(j).getControlPoint2() - curves3D.at(j).getEndPoint());
                    gradientVectorCp2 += (curves3D.at(j).getControlPoint2() - curves3D.at(j).getEndPoint());

                }

            }

        }

        gradient.push_back(gradientVectorStartPoint.x());
        gradient.push_back(gradientVectorStartPoint.y());
        gradient.push_back(gradientVectorStartPoint.z());

        gradient.push_back(gradientVectorCp1.x());
        gradient.push_back(gradientVectorCp1.y());
        gradient.push_back(gradientVectorCp1.z());

        gradient.push_back(gradientVectorCp2.x());
        gradient.push_back(gradientVectorCp2.y());
        gradient.push_back(gradientVectorCp2.z());

        gradient.push_back(gradientVectorEndPoint.x());
        gradient.push_back(gradientVectorEndPoint.y());
        gradient.push_back(gradientVectorEndPoint.z());

    }

    return gradient;

}

void Solver::setInitialGuess(std::vector<float> guess){

    this->initialGuess = guess;

}

float Solver::Cortho(const QVector<CubicBezierCurve>& curves3D){

    float value=0;

    for (int i=0;i<curves3D.size();i++) {

        for (int j=0;j<curves3D.size();j++) {

            if(i!=j){

                if(curves3D.at(i).getStartPoint() == curves3D.at(j).getStartPoint()){

                     value += QVector3D::dotProduct(curves3D.at(i).getControlPoint1() - curves3D.at(i).getStartPoint(),curves3D.at(j).getControlPoint1() - curves3D.at(j).getStartPoint());
                }

                if(curves3D.at(i).getStartPoint() == curves3D.at(j).getEndPoint()){

                    value += QVector3D::dotProduct(curves3D.at(i).getControlPoint1() - curves3D.at(i).getStartPoint(),curves3D.at(j).getControlPoint2() - curves3D.at(j).getEndPoint());

                }

                if(curves3D.at(i).getEndPoint() == curves3D.at(j).getStartPoint()){

                    value += QVector3D::dotProduct(curves3D.at(i).getControlPoint2() - curves3D.at(i).getEndPoint(),curves3D.at(j).getControlPoint1() - curves3D.at(j).getStartPoint());

                }

                if(curves3D.at(i).getEndPoint() == curves3D.at(j).getEndPoint()){

                    value += QVector3D::dotProduct(curves3D.at(i).getControlPoint2() - curves3D.at(i).getEndPoint(),curves3D.at(j).getControlPoint2() - curves3D.at(j).getEndPoint());

                }

            }

        }

    }

    return value;

}

int print_state (size_t iter, gsl_multiroot_fsolver * s)
{
    printf("iteration: %d\n x =(",iter);

    for(int i=0;i<s->x->size;i++){

        printf( "%f, ", gsl_vector_get (s->x, i));

    }

    printf(")\n f(x) =(");

    for(int i=0;i<s->x->size;i++){

        printf( "%f, ", gsl_vector_get (s->f, i));

    }

    printf(")\n");

}


void Solver::CreateBaselineReconstruction(const QVector3D& point){

    auto curvesToSolve = network.getSegmentsAtPoint(point);
    SolverParams params;
    params.segments = &curvesToSolve;
    params.selectedBaselinePoint = point;

    const gsl_multiroot_fsolver_type * T = gsl_multiroot_fsolver_hybrid;
    const gsl_multimin_fminimizer_type * mT = gsl_multimin_fminimizer_nmsimplex2;

    const size_t n = curvesToSolve.size();
    //const size_t curve_variable_number = n*4*3;
    const size_t curve_variable_number = 1;
    const size_t equation_size = curve_variable_number+1;
    gsl_vector *x = gsl_vector_alloc (equation_size);
    s = gsl_multiroot_fsolver_alloc (T, equation_size);

    gsl_multiroot_function f = {Solver::BaselineReconstructionFunction, equation_size, &params};
    std::cout << "solver started" << std::endl;
   /* for(int i=0; i < curve_variable_number; i+=12){

        CubicBezierCurve currentCurve = curvesToSolve.at(i/12);

        if(currentCurve.getStartPoint() == point){

            gsl_vector_set (x, i,   currentCurve.getStartPoint().x());
            gsl_vector_set (x, i+1, currentCurve.getStartPoint().y());
            gsl_vector_set (x, i+2, 100);
            gsl_vector_set (x, i+9, currentCurve.getEndPoint().x());
            gsl_vector_set (x, i+10,currentCurve.getEndPoint().y());
            gsl_vector_set (x, i+11,currentCurve.getEndPoint().z());

        }
        else{

            gsl_vector_set (x, i,   currentCurve.getStartPoint().x());
            gsl_vector_set (x, i+1, currentCurve.getStartPoint().y());
            gsl_vector_set (x, i+2, currentCurve.getStartPoint().z());
            gsl_vector_set (x, i+9, currentCurve.getEndPoint().x());
            gsl_vector_set (x, i+10,currentCurve.getEndPoint().y());
            gsl_vector_set (x, i+11,1000);

        }

        gsl_vector_set (x, i+3, currentCurve.getControlPoint1().x());
        gsl_vector_set (x, i+4, currentCurve.getControlPoint1().y());
        gsl_vector_set (x, i+5, currentCurve.getControlPoint1().z());
        gsl_vector_set (x, i+6, currentCurve.getControlPoint2().x());
        gsl_vector_set (x, i+7, currentCurve.getControlPoint2().y());
        gsl_vector_set (x, i+8, currentCurve.getControlPoint2().z());

    }
*/
    gsl_vector_set (x, 0 , initialGuess.at(0));
    gsl_vector_set (x, equation_size-1 ,initialGuess.at(1)/1000.0f );

    gsl_multiroot_fsolver_set (s, &f, x);

    int status;
    size_t iter=0;

    do{
        iter++;
        status = gsl_multiroot_fsolver_iterate (s);

        if (status)   /* check if solver is stuck */
            break;
        print_state(iter,s);

        status = gsl_multiroot_test_residual (s->f, 1e-7);
    }while(iter < 10 && status == GSL_CONTINUE);
/*
    QVector<CubicBezierCurve> results;

    for(int i=0; i < curve_variable_number; i+=12){

        QVector3D start( gsl_vector_get (s->x, i), gsl_vector_get (s->x, i+1), gsl_vector_get (s->x, i+2));
        QVector3D ControlPoint1(gsl_vector_get (s->x, i+3), gsl_vector_get (s->x, i+4), gsl_vector_get (s->x, i+5));
        QVector3D ControlPoint2(gsl_vector_get (s->x, i+6), gsl_vector_get (s->x, i+7), gsl_vector_get (s->x, i+8));
        QVector3D end(gsl_vector_get (s->x, i+9), gsl_vector_get (s->x, i+10), gsl_vector_get (s->x, i+11));

        CubicBezierCurve currentResult(start,ControlPoint1,ControlPoint2,end);
        results.push_back(currentResult);

    }

    network.ReplaceSegmentsAtPoint(point,results);
*/
    gsl_multiroot_fsolver_free(s);

    std::cout << "solver finished" << std::endl;

}

BezierCurveNetwork Solver::getCurrentSolution() const{

    return this->network;

}

float Solver::Eaccuracy(const QVector<CubicBezierCurve>& originalCurves, const QVector<CubicBezierCurve>& curves3D){

    float projectionAccuracy = 0.0f;

    for (int i=0; i<originalCurves.size() ; i++ ) {

        float length1 = (curves3D.at(i).getStartPoint().toVector2D() - originalCurves.at(i).getStartPoint().toVector2D() ).length();
        float length2 = (curves3D.at(i).getControlPoint1().toVector2D() - originalCurves.at(i).getControlPoint1().toVector2D() ).length();
        float length3 = (curves3D.at(i).getControlPoint2().toVector2D() - originalCurves.at(i).getControlPoint2().toVector2D() ).length();
        float length4 = (curves3D.at(i).getEndPoint().toVector2D() - originalCurves.at(i).getEndPoint().toVector2D() ).length();

        projectionAccuracy += length1*length1 + length2*length2 + length3*length3 + length4*length4;

    }

    for (int i=0; i<curves3D.size()-1 ; i++ ) {

        QVector2D d1 = originalCurves.at(i).getControlPoint1().toVector2D() - originalCurves.at(i).getStartPoint().toVector2D();
        QVector2D d2 = originalCurves.at(i).getControlPoint2().toVector2D() - originalCurves.at(i).getControlPoint1().toVector2D();
        QVector2D d3 = originalCurves.at(i).getEndPoint().toVector2D() - originalCurves.at(i).getControlPoint2().toVector2D();

        float length1 = (curves3D.at(i).getControlPoint1().toVector2D() - curves3D.at(i).getStartPoint().toVector2D() - d1 ).length();
        float length2 = (curves3D.at(i).getControlPoint2().toVector2D() - curves3D.at(i).getControlPoint1().toVector2D() -  d2 ).length();
        float length3 = (curves3D.at(i).getEndPoint().toVector2D() - curves3D.at(i).getControlPoint2().toVector2D() - d3 ).length();

        projectionAccuracy += Wd(d1.length())*length1*length1 + Wd(d2.length())*length2*length2 + Wd(d3.length())*length3*length3;

    }

    return projectionAccuracy;

}

std::vector<float> Solver::EaccuracyGrad(const QVector<CubicBezierCurve>& originalCurves, const QVector<CubicBezierCurve>& curves3D){

    std::vector<float> gradientVector;

    for (int i=0; i<originalCurves.size() ; i++ ) {

        QVector2D v1 = 2.0f*(curves3D.at(i).getStartPoint().toVector2D() - originalCurves.at(i).getStartPoint().toVector2D());
        QVector2D v2 = 2.0f*(curves3D.at(i).getControlPoint1().toVector2D() - originalCurves.at(i).getControlPoint1().toVector2D());
        QVector2D v3 = 2.0f*(curves3D.at(i).getControlPoint2().toVector2D() - originalCurves.at(i).getControlPoint2().toVector2D());
        QVector2D v4 = 2.0f*(curves3D.at(i).getEndPoint().toVector2D() - originalCurves.at(i).getEndPoint().toVector2D());

        QVector2D d1 = originalCurves.at(i).getControlPoint1().toVector2D() - originalCurves.at(i).getStartPoint().toVector2D();
        QVector2D d2 = originalCurves.at(i).getControlPoint2().toVector2D() - originalCurves.at(i).getControlPoint1().toVector2D();
        QVector2D d3 = originalCurves.at(i).getEndPoint().toVector2D() - originalCurves.at(i).getControlPoint2().toVector2D();

        QVector2D difference1 = ( curves3D.at(i).getControlPoint1().toVector2D() - curves3D.at(i).getStartPoint().toVector2D() - d1 );
        QVector2D difference2 = ( curves3D.at(i).getControlPoint2().toVector2D() - curves3D.at(i).getControlPoint1().toVector2D() -  d2 );
        QVector2D difference3 = ( curves3D.at(i).getEndPoint().toVector2D() - curves3D.at(i).getControlPoint2().toVector2D() - d3 );

        //push back every calculated dimension of gradient
        //k=0
        QVector2D tempVector1 = 2*difference1;
        gradientVector.push_back( v1.x() + Wd(d1.length())*tempVector1.x() );
        gradientVector.push_back( v1.y() + Wd(d1.length())*tempVector1.y() );
        gradientVector.push_back( 0 );

        //k=1
        tempVector1 = 2*difference1;
        QVector2D tempVector2 = 2*difference2;
        gradientVector.push_back(v2.x() + Wd(d1.length())*tempVector1.x() + Wd(d2.length())*tempVector2.x() );
        gradientVector.push_back(v2.y() + Wd(d1.length())*tempVector1.y() + Wd(d2.length())*tempVector2.y() );
        gradientVector.push_back(0);

        //k=2
        tempVector1 = 2*difference2;
        tempVector2 = 2*difference3;
        gradientVector.push_back(v3.x() + Wd(d2.length())*tempVector1.x() + Wd(d3.length())*tempVector2.x() );
        gradientVector.push_back(v3.y() + Wd(d2.length())*tempVector1.y() + Wd(d3.length())*tempVector2.y() );
        gradientVector.push_back(0);

        //k=3
        tempVector1 = 2*difference3;
        gradientVector.push_back(v4.x() + Wd(d3.length())*tempVector1.x());
        gradientVector.push_back(v4.y() + Wd(d3.length())*tempVector1.y());
        gradientVector.push_back(0);


    }

    return gradientVector;

}

float getLongestPathBetweenPoints(const QVector3D& pt0_orig,const QVector3D& pt1_orig, const QVector3D& pt2_orig){

    std::vector<float> lengthToSort;

    lengthToSort.push_back((pt0_orig-pt1_orig).length());
    lengthToSort.push_back((pt0_orig-pt2_orig).length());
    lengthToSort.push_back((pt1_orig-pt2_orig).length());

    std::sort(lengthToSort.begin(),lengthToSort.end());

    return lengthToSort.back();

}

float getLongestPathBetweenPoints(const QVector3D& pt0_orig,const QVector3D& pt1_orig, const QVector3D& pt2_orig, const QVector3D& pt3_orig){

    std::vector<float> lengthToSort;

    lengthToSort.push_back((pt0_orig-pt1_orig).length());
    lengthToSort.push_back((pt0_orig-pt2_orig).length());
    lengthToSort.push_back((pt0_orig-pt3_orig).length());
    lengthToSort.push_back((pt1_orig-pt2_orig).length());
    lengthToSort.push_back((pt1_orig-pt3_orig).length());
    lengthToSort.push_back((pt2_orig-pt3_orig).length());

    std::sort(lengthToSort.begin(),lengthToSort.end());

    return lengthToSort.back();

}

std::vector<float> Solver::EvariationPairGrad(const CubicBezierCurve& firstOriginal, const CubicBezierCurve& secondOriginal, const CubicBezierCurve& first3D, const CubicBezierCurve& second3D){

    std::vector<float> grad;

    grad.push_back(0);
    grad.push_back(0);
    grad.push_back(0);

    Eigen::Matrix3f A;
    Eigen::Matrix3f collinearity;

    Eigen::Vector3f b;

    A << firstOriginal.getControlPoint1().x(), firstOriginal.getControlPoint1().y() ,1,  firstOriginal.getControlPoint2().x(), firstOriginal.getControlPoint2().y(),1,  secondOriginal.getControlPoint1().x(), secondOriginal.getControlPoint2().y(),1;
    collinearity << first3D.getControlPoint1().x(), first3D.getControlPoint1().y() ,1,  first3D.getControlPoint2().x(), first3D.getControlPoint2().y(),1,  second3D.getControlPoint1().x(), second3D.getControlPoint2().y(),1;
    b << secondOriginal.getControlPoint2().x(), secondOriginal.getControlPoint2().y(), 1;
    Eigen::ColPivHouseholderQR<Eigen::Matrix3f> dec(A.inverse());

    //x contains q1, q2, q3 numbers
    Eigen::Vector3f q = dec.solve(b);

    QVector3D gradient;
    float w;

    //TODO: take other triplet into account

    if(std::abs(collinearity.determinant()) < 0.05){
        //three points are collinear

        float t0 = (firstOriginal.getControlPoint2().x() - secondOriginal.getControlPoint1().x())/(firstOriginal.getControlPoint1().x()+secondOriginal.getControlPoint1().x());

        gradient = t0*firstOriginal.getControlPoint1()+(1-t0)*secondOriginal.getControlPoint1()-firstOriginal.getControlPoint2();
        w = Wd(getLongestPathBetweenPoints(firstOriginal.getControlPoint1(),firstOriginal.getControlPoint2(),secondOriginal.getControlPoint1()));

        grad.push_back(2*w*gradient.x()*t0);
        grad.push_back(2*w*gradient.y()*t0);
        grad.push_back(2*w*gradient.z()*t0);

        grad.push_back(2*w*gradient.x());
        grad.push_back(2*w*gradient.y());
        grad.push_back(2*w*gradient.z());

    }
    else{
        //three points are not collinear

        gradient = q.x() * first3D.getControlPoint1() + q.y()* first3D.getControlPoint2() + q.z()*second3D.getControlPoint1() - second3D.getControlPoint2();
        w = Wd(getLongestPathBetweenPoints(first3D.getControlPoint1(), first3D.getControlPoint2(), second3D.getControlPoint1(), second3D.getControlPoint2()));

        grad.push_back(2*w*gradient.x()*q.x());
        grad.push_back(2*w*gradient.y()*q.x());
        grad.push_back(2*w*gradient.z()*q.x());

        grad.push_back(2*w*gradient.x()*q.y());
        grad.push_back(2*w*gradient.y()*q.y());
        grad.push_back(2*w*gradient.z()*q.y());


    }


    grad.push_back(0);
    grad.push_back(0);
    grad.push_back(0);


    return grad;

}

std::vector<float> Solver::EvariationSelfGrad(const CubicBezierCurve& curveOriginal, const CubicBezierCurve& curve3D){

    std::vector<float> grad;

    Eigen::Matrix3f A;
    Eigen::Matrix3f collinearity;

    Eigen::Vector3f b;

    A << curveOriginal.getStartPoint().x(), curveOriginal.getStartPoint().y() ,1,  curveOriginal.getControlPoint1().x(), curveOriginal.getControlPoint1().y(),1,  curve3D.getControlPoint2().x(), curve3D.getControlPoint2().y(),1;
    collinearity << curve3D.getStartPoint().x(), curve3D.getStartPoint().y() ,1,  curve3D.getControlPoint1().x(), curve3D.getControlPoint1().y(),1,  curve3D.getControlPoint2().x(), curve3D.getControlPoint2().y(),1;
    b << curveOriginal.getEndPoint().x(), curveOriginal.getEndPoint().y(), 1;
    Eigen::ColPivHouseholderQR<Eigen::Matrix3f> dec(A.inverse());

    //x contains q1, q2, q3 numbers
    Eigen::Vector3f q = dec.solve(b);

    QVector3D gradient;
    float w;

    //TODO: take other triplet into account

    if(std::abs(collinearity.determinant()) < 0.05){
        //three points are collinear

        float t0 = (curve3D.getControlPoint1().x() - curve3D.getControlPoint2().x())/(curve3D.getStartPoint().x()+curve3D.getControlPoint2().x());

        gradient = t0*curve3D.getStartPoint()+(1-t0)*curve3D.getControlPoint2()-curve3D.getControlPoint1();
        w = Wd(getLongestPathBetweenPoints(curve3D.getStartPoint(),curve3D.getControlPoint1(),curve3D.getControlPoint2()));

        grad.push_back(2*w*gradient.x()*t0);
        grad.push_back(2*w*gradient.y()*t0);
        grad.push_back(2*w*gradient.z()*t0);

        grad.push_back(2*w*gradient.x()*-1);
        grad.push_back(2*w*gradient.y()*-1);
        grad.push_back(2*w*gradient.z()*-1);

        grad.push_back(2*w*gradient.x()*(1-t0));
        grad.push_back(2*w*gradient.y()*(1-t0));
        grad.push_back(2*w*gradient.z()*(1-t0));

        grad.push_back(0);
        grad.push_back(0);
        grad.push_back(0);

    }
    else{
        //three points are not collinear

        gradient = q.x() * curve3D.getEndPoint() + q.y()* curve3D.getControlPoint1() + q.z()*curve3D.getControlPoint2() - curve3D.getEndPoint();
        w = Wd(getLongestPathBetweenPoints(curve3D.getStartPoint(), curve3D.getEndPoint(), curve3D.getControlPoint1(), curve3D.getControlPoint2()));

        grad.push_back(2*w*gradient.x()*q.x());
        grad.push_back(2*w*gradient.y()*q.x());
        grad.push_back(2*w*gradient.z()*q.x());

        grad.push_back(2*w*gradient.x()*q.y());
        grad.push_back(2*w*gradient.y()*q.y());
        grad.push_back(2*w*gradient.z()*q.y());

        grad.push_back(2*w*gradient.x()*q.z());
        grad.push_back(2*w*gradient.y()*q.z());
        grad.push_back(2*w*gradient.z()*q.z());

        grad.push_back(2*w*gradient.x()*-1);
        grad.push_back(2*w*gradient.y()*-1);
        grad.push_back(2*w*gradient.z()*-1);



    }

    return grad;

}

float Solver::EvariationCPT(const QVector3D& pt0_orig,const QVector3D& pt1_orig, const QVector3D& pt2_orig, const QVector3D& pt3_orig,const QVector3D& pt0,const QVector3D& pt1, const QVector3D& pt2, const QVector3D& pt3){

    Eigen::Matrix3f A;
    Eigen::Vector3f b;

    A << pt0_orig.x(), pt0_orig.y() ,1,  pt1_orig.x(),pt1_orig.y(),1,  pt2_orig.x(),pt2_orig.y(),1;
    b << pt3_orig.x(), pt3_orig.y(), 1;
    Eigen::ColPivHouseholderQR<Eigen::Matrix3f> dec(A.inverse());

    //x contains q1, q2, q3 numbers
    Eigen::Vector3f q = dec.solve(b);

    QVector3D affineComb;
    float w;

    if(std::abs(A.determinant()) < 0.05){
        //three points are collinear

        float t0 = (pt1_orig.x() - pt2_orig.x())/(pt0_orig.x()+pt2_orig.x());

        affineComb = t0*pt0+(1-t0)*pt2-pt1;
        w = Wd(getLongestPathBetweenPoints(pt0_orig,pt1_orig,pt2_orig));


    }
    else{
        //three points are not collinear

        affineComb = q.x() * pt0 + q.y()* pt1 + q.z()*pt2 - pt3;
        w = Wd(getLongestPathBetweenPoints(pt0_orig,pt1_orig,pt2_orig,pt3_orig));

    }

    return affineComb.lengthSquared() * w;

}

std::vector<float> Solver::EvariationGrad(const QVector<CubicBezierCurve>& originalJoiningCurves, const QVector<CubicBezierCurve>& curvesJoining_3D){

    std::vector<float> gradient;

    for(int i = 0; i < originalJoiningCurves.size();i++){

        std::vector<float> selfComponent = EvariationSelfGrad(originalJoiningCurves.at(i), curvesJoining_3D.at(i));
        std::vector<float> pairComponent(12);

        for (int j=i+1; j < originalJoiningCurves.size() ; j++) {

            std::vector<float> currentPairComponent = EvariationPairGrad(originalJoiningCurves.at(i), originalJoiningCurves.at(j), curvesJoining_3D.at(i), curvesJoining_3D.at(j));

            for (int k=0;k<12;k++) {

                pairComponent.at(k) += currentPairComponent.at(k);

            }

        }

        for (int k=0;k<12;k++) {

            gradient.push_back(selfComponent.at(k) + pairComponent.at(k));

        }

    }

    return gradient;

}

float Solver::Evariation(const QVector<CubicBezierCurve>& originalJoiningCurves, const QVector<CubicBezierCurve>& curvesJoining_3D){

    float variation = 0.0f;

    for (int i=0; i < originalJoiningCurves.size(); i++ ) {

        CubicBezierCurve originalBezierNumI = originalJoiningCurves.at(i);
        CubicBezierCurve bezierNumI = originalJoiningCurves.at(i);

        variation += EvariationCPT(originalBezierNumI.getStartPoint(), originalBezierNumI.getControlPoint1(), originalBezierNumI.getControlPoint2(), originalBezierNumI.getEndPoint(),bezierNumI.getStartPoint(),bezierNumI.getControlPoint1(), bezierNumI.getControlPoint2(), bezierNumI.getEndPoint());

        for (int j=i+1; j < originalJoiningCurves.size(); j++ ) {

            CubicBezierCurve originalBezierNumJ = originalJoiningCurves.at(j);
            CubicBezierCurve bezierNumJ = originalJoiningCurves.at(j);

            variation += EvariationCPT(originalBezierNumI.getControlPoint1(), originalBezierNumI.getControlPoint2(), originalBezierNumJ.getControlPoint1(), originalBezierNumJ.getControlPoint2(), bezierNumI.getControlPoint1(), bezierNumI.getControlPoint2(), bezierNumJ.getControlPoint1(), bezierNumJ.getControlPoint2());

        }

    }

    return variation;

}

//This is calculated to (1 , 0) (2 , 1) (3 , 2) pairs think about it if its good
float Solver::Eforeshortening(const QVector<CubicBezierCurve>& originalCurves, const QVector<CubicBezierCurve>& curves3D){

    float foreshortening = 0.0;

    for(int i=0; i < originalCurves.size(); i++){

        float fs1 = curves3D.at(i).getControlPoint1().z() - curves3D.at(i).getStartPoint().z();
        float fs2 = curves3D.at(i).getControlPoint2().z() - curves3D.at(i).getControlPoint1().z();
        float fs3 = curves3D.at(i).getEndPoint().z() - curves3D.at(i).getControlPoint2().z();

        QVector2D d1 = originalCurves.at(i).getControlPoint1().toVector2D() - originalCurves.at(i).getStartPoint().toVector2D();
        QVector2D d2 = originalCurves.at(i).getControlPoint2().toVector2D() - originalCurves.at(i).getControlPoint1().toVector2D();
        QVector2D d3 = originalCurves.at(i).getEndPoint().toVector2D() - originalCurves.at(i).getControlPoint2().toVector2D();

        foreshortening += Wd(d1.length())*fs1*fs1 + Wd(d2.length())*fs2*fs2 + Wd(d3.length())*fs3*fs3;

    }

}

//This is calculated to (1 , 0) (2 , 1) (3 , 2) pairs think about it if its good
std::vector<float> Solver::EforeshorteningGrad(const QVector<CubicBezierCurve>& originalCurves, const QVector<CubicBezierCurve>& curves3D){

    std::vector<float> gradientVector;

    for(int i=0; i < originalCurves.size(); i++){

        float fs1 = curves3D.at(i).getControlPoint1().z() - curves3D.at(i).getStartPoint().z();
        float fs2 = curves3D.at(i).getControlPoint2().z() - curves3D.at(i).getControlPoint1().z();
        float fs3 = curves3D.at(i).getEndPoint().z() - curves3D.at(i).getControlPoint2().z();

        QVector2D d1 = originalCurves.at(i).getControlPoint1().toVector2D() - originalCurves.at(i).getStartPoint().toVector2D();
        QVector2D d2 = originalCurves.at(i).getControlPoint2().toVector2D() - originalCurves.at(i).getControlPoint1().toVector2D();
        QVector2D d3 = originalCurves.at(i).getEndPoint().toVector2D() - originalCurves.at(i).getControlPoint2().toVector2D();

        //gradient by i=0
        gradientVector.push_back( 0 );
        gradientVector.push_back( 0 );
        gradientVector.push_back( Wd(d1.length())*2*fs1);

        //gradient by i=1
        gradientVector.push_back( 0 );
        gradientVector.push_back( 0 );
        gradientVector.push_back( (Wd(d1.length())*fs1 + Wd(d2.length())*fs2)*2 );

        //gradient by i=2
        gradientVector.push_back( 0 );
        gradientVector.push_back( 0 );
        gradientVector.push_back( (Wd(d2.length())*fs2 + Wd(d3.length())*fs3)*2 );

        //gradient by i=3
        gradientVector.push_back( 0 );
        gradientVector.push_back( 0 );
        gradientVector.push_back( Wd(d3.length())*fs3*2 );

    }

    return gradientVector;

}

float Solver::Ctangent(const CubicBezierCurve& c1_3D, CubicBezierCurve& c2_3D){

    return QVector3D::dotProduct(c1_3D.getStartPoint()-c1_3D.getControlPoint1(), c2_3D.getStartPoint()-c2_3D.getControlPoint1());

}

QVector3D Solver::Cparallel(const CubicBezierCurve& c1_original, CubicBezierCurve& c2_original,const CubicBezierCurve& c1_3D, CubicBezierCurve& c2_3D){

    return (c1_3D.getControlPoint1() - c1_3D.getStartPoint())/(c1_original.getControlPoint1()-c1_original.getStartPoint()).length() - (c1_3D.getControlPoint1() - c2_3D.getStartPoint())/(c2_original.getControlPoint1()-c2_original.getStartPoint()).length();

}

float Solver::Wd(float d){

    float sketchboundingboxdiagonalsquare = 400.0f;

    return std::exp(-d*d/(2*(sketchboundingboxdiagonalsquare)))+0.01;

}

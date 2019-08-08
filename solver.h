#ifndef SOLVER_H
#define SOLVER_H

#include "beziercurvenetwork.h"
#include "Eigen/Core"
#include "Eigen/QR"
#include "Eigen/Dense"
#include "gsl/gsl_multiroots.h"
#include "gsl/gsl_multimin.h"

struct SolverParams{

    QVector<CubicBezierCurve>* segments;
    QVector3D selectedBaselinePoint;

};

class Solver{

private:
    BezierCurveNetwork network;
    BezierCurveNetwork solution;
    gsl_multiroot_fsolver * s;
    std::vector<float> initialGuess;

    static std::vector<float> EaccuracyGrad(const QVector<CubicBezierCurve>& originalCurves, const QVector<CubicBezierCurve>& curves3D);
    static float Eaccuracy(const QVector<CubicBezierCurve>& originalCurves, const QVector<CubicBezierCurve>& curves3D);
    static std::vector<float> EvariationGrad(const QVector<CubicBezierCurve>& originalJoiningCurves, const QVector<CubicBezierCurve>& curvesJoining_3D);
    static float Evariation(const QVector<CubicBezierCurve>& originalJoiningCurves, const QVector<CubicBezierCurve>& curvesJoining_3D);
    static float EvariationCPT(const QVector3D& pt0_orig,const QVector3D& pt1_orig, const QVector3D& pt2_orig, const QVector3D& pt3_orig,const QVector3D& pt0,const QVector3D& pt1, const QVector3D& pt2, const QVector3D& pt3);
    static std::vector<float> EvariationPairGrad(const CubicBezierCurve& firstOriginal, const CubicBezierCurve& secondOriginal, const CubicBezierCurve& first3D, const CubicBezierCurve& second3D);
    static std::vector<float> EvariationSelfGrad(const CubicBezierCurve& curveOriginal, const CubicBezierCurve& curve3D);
    static std::vector<float> EforeshorteningGrad(const QVector<CubicBezierCurve>& originalCurves, const QVector<CubicBezierCurve>& curves3D);
    static float Eforeshortening(const QVector<CubicBezierCurve>& originalCurves, const QVector<CubicBezierCurve>& curves3D);
    static float Ctangent(const CubicBezierCurve& c1, CubicBezierCurve& c2);
    static QVector3D Cparallel(const CubicBezierCurve& c1_original, CubicBezierCurve& c2_original,const CubicBezierCurve& c1_3D, CubicBezierCurve& c2_3D);
    static float Wd(float);
    static std::vector<float> CorthoGrad(const QVector<CubicBezierCurve>& curves3D);
    static float Cortho(const QVector<CubicBezierCurve>& curves3D);
    static int BaselineReconstructionFunction(const gsl_vector * x, void *params, gsl_vector * f);
public:
    Solver();
    void setNetwork(const BezierCurveNetwork& net);
    void CreateBaselineReconstruction(const QVector3D& point);
    void setInitialGuess(std::vector<float> guess);
    BezierCurveNetwork getCurrentSolution() const;

};

#endif // SOLVER_H

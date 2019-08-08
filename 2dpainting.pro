QT          += widgets
QT *= xml opengl


HEADERS     = \
              window.h \
    cubicbeziercurve.h \
    nanosvg.h \
    curvedrawer3d.h \
    CurveDrawer2D.h \
    solver.h \
    beziercurvenetwork.h
SOURCES     = \
              main.cpp \
              window.cpp \
    cubicbeziercurve.cpp \
    CurveDrawer2D.cpp \
    curvedrawer3d.cpp \
    solver.cpp \
    beziercurvenetwork.cpp

# install
target.path = /home/ferenc/BME/Onlab1/2dpainting
INSTALLS += target
LIBS *= -lQGLViewer -lgsl -lgslcblas -lm
INCLUDEPATH = /usr/include/gsl d:/stl/include
DISTFILES += \
    Curve2ModelForm.ui.qml \
    Curve2Model.qml

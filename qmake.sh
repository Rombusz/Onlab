#!/bin/bash
/opt/Qt/5.12.1/gcc_64/bin/qmake /home/ferenc/BME/Onlab1/2dpainting_vscode/2dpainting.pro -spec linux-g++ CONFIG+=debug CONFIG+=qml_debug
make -f /home/ferenc/BME/Onlab1/2dpainting_vscode/Build/Makefile qmake_all
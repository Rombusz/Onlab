#!/bin/bash
qmake /home/ferenc/BME/Onlab1/2dpainting/2dpainting.pro -spec linux-g++ CONFIG+=debug CONFIG+=qml_debug
make -f /home/ferenc/BME/Onlab1/2dpainting/Build/Makefile qmake_all
#-------------------------------------------------
#
# Project created by QtCreator 2016-01-18T14:59:16
#
#-------------------------------------------------

greaterThan(QT_MAJOR_VERSION, 5): QT += widgets

TARGET = newGUI
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    savedialog.cpp \
    popup.cpp \
    myTensor.cpp

HEADERS  += mainwindow.h \
    savedialog.h \
    myTensor.h \
    popup.h

FORMS    += mainwindow.ui \
    savedialog.ui

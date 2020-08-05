#-------------------------------------------------
#
# Project created by QtCreator 2020-08-05T14:30:43
#
#-------------------------------------------------

VERSION = 0.0.1.0
QMAKE_TARGET_COMPANY = WIN-DESIGN
QMAKE_TARGET_PRODUCT = monkeyRadio
QMAKE_TARGET_DESCRIPTION = monkeyRadio
QMAKE_TARGET_COPYRIGHT = (c) 2020 WIN-DESIGN

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

win32-g++ {
    message("mingw")
    INCLUDEPATH += C:\dev\3rdParty\monkeyboard\include
    LIBS += -LC:\dev\3rdParty\monkeyboard\lib -lkeystonecomm
}

unix {
    message("*nix")
    INCLUDEPATH += /data/dab+/keystonecomm/KeyStoneCOMM
    LIBS += -lkeystonecomm
}

TARGET = monkeyRadio
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        cmainwindow.cpp

HEADERS += \
        cmainwindow.h

FORMS += \
        cmainwindow.ui

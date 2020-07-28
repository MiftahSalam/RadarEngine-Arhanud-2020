#-------------------------------------------------
#
# Project created by QtCreator 2020-07-20T13:25:26
#
#-------------------------------------------------

QT       += widgets network opengl

include(../../build.pri)
include(RadarEngine.pri)

contains(DEFINES, RADAR_ENGINE_ARND_STATIC) {
    message(Building static radar-engine...)
    CONFIG += staticlib
}

TARGET = RadarEngine-arnd
TEMPLATE = lib
DESTDIR = ../bin
DEFINES += RADAR_ENGINE_LIBRARY
MOC_DIR = tmp
OBJECTS_DIR = obj

unix {
    target.path = /usr/lib
    INSTALLS += target
}

header_base.files = $$HEADERS_BASE
header_base.path = /usr/include/RadarEngine
INSTALLS += header_base


unix:!macx: LIBS += -L/usr/lib/ -llog4qt
unix:!macx: LIBS += -L/usr/lib/Crypto/ -lCrypto

INCLUDEPATH += /include/
DEPENDPATH += /include/

INCLUDEPATH += /usr/include/Crypto
DEPENDPATH += /usr/include/Crypto

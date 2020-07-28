HEADERS_BASE += \
                $$PWD/radarengine_global.h \
                $$PWD/radarengine.h \
                $$PWD/radarreceive.h \
                $$PWD/radartransmit.h \
                $$PWD/radardraw.h \
                $$PWD/arpatarget.h \
                $$PWD/kalmanfilter.h \
                $$PWD/radararpa.h

SOURCES += \
           $$PWD/radarengine.cpp \ 
    $$PWD/radarreceive.cpp \
    $$PWD/radartransmit.cpp \
    $$PWD/radardraw.cpp \
    $$PWD/arpatarget.cpp \
    $$PWD/kalmanfilter.cpp \
    $$PWD/radararpa.cpp

HEADERS += $$HEADERS_BASE \

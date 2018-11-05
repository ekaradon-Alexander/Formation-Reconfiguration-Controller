TEMPLATE = lib
CONFIG += console c++11 staticlib
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    main.cpp \
    onConnectionRecvd.cpp \
    onControlMsgPredLocRecvd.cpp \
    onControlRequestRecvd.cpp \
    onMissionRecvd.cpp

HEADERS += \
    controllercommunication.h \
    handlers.h \
    main.h

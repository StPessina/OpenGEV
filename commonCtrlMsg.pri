@INCLUDE += commonCtrlMsg

HEADERS += \
    $$PWD/privilege.h \
    $$PWD/abstractcommand.h \
    $$PWD/abstractmessagehandler.h \
    $$PWD/abstractmessagehandlerfactory.h \
    $$PWD/cmdnotsupportedmh.h

SOURCES += \
    $$PWD/abstractcommand.cpp \
    $$PWD/abstractmessagehandler.cpp \
    $$PWD/abstractmessagehandlerfactory.cpp \
    $$PWD/cmdnotsupportedmh.cpp

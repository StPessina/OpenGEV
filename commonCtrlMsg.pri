@INCLUDE += commonCtrlMsg

HEADERS += \
    $$PWD/privilege.h \
    $$PWD/abstractcommand.h \
    $$PWD/abstractmessagehandler.h \
    $$PWD/abstractmessagehandlerfactory.h

SOURCES += \
    $$PWD/abstractcommand.cpp \
    $$PWD/abstractmessagehandler.cpp \
    $$PWD/abstractmessagehandlerfactory.cpp

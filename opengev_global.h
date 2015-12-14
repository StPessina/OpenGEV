#ifndef OPENGEV_GLOBAL_H
#define OPENGEV_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(OPENGEV_LIBRARY)
#  define OPENGEVSHARED_EXPORT Q_DECL_EXPORT
#else
#  define OPENGEVSHARED_EXPORT Q_DECL_IMPORT
#endif

#define CONTROL_CHANNEL_DEF_PORT 3956

#define SPEC_VERSION_MINOR 2
#define SPEC_VERSION_MAJOR 2

//#define USE_LOG4CPP 1

//#define USE_QT_SOCKET 0
#define USE_BOOST_SOCKET 0
//#define USE_OSAPI_SOCKET 0

//This take effect only with boost socket
#define ENABLE_BOOST_ASYNCH_SOCKET 1

#define SOCKET_ERROR "Multiple socket enabled at the same time. Choose one."

#if (defined(USE_QT_SOCKET) && defined(USE_BOOST_SOCKET)) || (defined(USE_QT_SOCKET) && defined(USE_OSAPI_SOCKET)) || (defined(USE_OSAPI_SOCKET) && defined(USE_BOOST_SOCKET))
    #error SOCKET_ERROR
#endif

#endif // OPENGEV_GLOBAL_H

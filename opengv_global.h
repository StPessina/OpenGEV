#ifndef OPENGV_GLOBAL_H
#define OPENGV_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(OPENGV_LIBRARY)
#  define OPENGVSHARED_EXPORT Q_DECL_EXPORT
#else
#  define OPENGVSHARED_EXPORT Q_DECL_IMPORT
#endif

#define CONTROL_CHANNEL_DEF_PORT 3956

#define SPEC_VERSION_MINOR 2
#define SPEC_VERSION_MAJOR 2

//#define USE_LOG4CPP 1

//#define USE_QT_SOCKET 0
//#define USE_BOOST_SOCKET 0
#define USE_OSAPI_SOCKET 0

#define SOCKET_ERROR "Multiple socket enabled at the same time. Choose one."

#if (defined(USE_QT_SOCKET) && defined(USE_BOOST_SOCKET)) || (defined(USE_QT_SOCKET) && defined(USE_OSAPI_SOCKET)) || (defined(USE_OSAPI_SOCKET) && defined(USE_BOOST_SOCKET))
    #error SOCKET_ERROR
#endif

#endif // OPENGV_GLOBAL_H

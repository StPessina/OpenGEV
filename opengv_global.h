#ifndef OPENGV_GLOBAL_H
#define OPENGV_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(OPENGV_LIBRARY)
#  define OPENGVSHARED_EXPORT Q_DECL_EXPORT
#else
#  define OPENGVSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // OPENGV_GLOBAL_H

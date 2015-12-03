#ifndef OPENGV_H
#define OPENGV_H

#include "opengv_global.h"
#include "iostream"

#ifdef USE_LOG4CPP
    #include <log4cpp/PropertyConfigurator.hh>
#endif

class OPENGVSHARED_EXPORT OpenGV
{

public:
    OpenGV();

    static void configure();

private:
#ifdef USE_LOG4CPP
    static void initLog4cpp();
#endif
};

#endif // OPENGV_H

#ifndef OPENGEV_H
#define OPENGEV_H

#include "opengev_global.h"
#include "iostream"

#ifdef USE_LOG4CPP
    #include <log4cpp/PropertyConfigurator.hh>
#endif

class OPENGEVSHARED_EXPORT OpenGEV
{

public:
    OpenGEV();

    static void configure();

private:
#ifdef USE_LOG4CPP
    static void initLog4cpp();
#endif
};

#endif // OPENGEV_H

#ifndef OPENGV_H
#define OPENGV_H

#include "opengv_global.h"
#include "iostream"

#include <log4cpp/PropertyConfigurator.hh>

class OPENGVSHARED_EXPORT OpenGV
{

public:
    OpenGV();

    static void configure();

private:
    static void initLog4cpp();
};

#endif // OPENGV_H

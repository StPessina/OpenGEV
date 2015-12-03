#ifndef GVCOMPONENT_H
#define GVCOMPONENT_H

#include "opengv_global.h"

#ifdef USE_LOG4CPP
    #include <log4cpp/Category.hh>
#endif

/**
 * @brief The GVComponent class genically define components that can be found on GigE vision network
 */
class GVComponent
{
public:
    /**
     * @brief GVComponent constructor
     */
    GVComponent();

    /**
     * @brief ~GVComponent deconstructor
     */
    virtual ~GVComponent() {}

protected:

#ifdef USE_LOG4CPP
    /**
     * @brief logger
     */
    //log4cpp::Category &logger = log4cpp::Category::getInstance("ComponentLog");
#endif

};

#endif // GVCOMPONENT_H

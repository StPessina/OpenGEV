#ifndef GVCOMPONENT_H
#define GVCOMPONENT_H

#include <log4cpp/Category.hh>

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

    /**
     * @brief logger
     */
    //log4cpp::Category &logger = log4cpp::Category::getInstance("ComponentLog");

};

#endif // GVCOMPONENT_H

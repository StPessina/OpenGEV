#ifndef GVCOMPONENT_H
#define GVCOMPONENT_H

#include <log4cpp/Category.hh>

class GVComponent
{
public:
    GVComponent();
    virtual ~GVComponent() {}

protected:

    log4cpp::Category &logger = log4cpp::Category::getInstance("ComponentLog");

};

#endif // GVCOMPONENT_H

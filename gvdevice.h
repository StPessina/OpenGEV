#ifndef GVDEVICE_H
#define GVDEVICE_H

#include <string>

#include "gvcomponent.h"

class GVDevice : public GVComponent
{
public:
    GVDevice(std::string manufacture_name, std::string model_name, std::string device_name);

    std::string getManufactureName();

    std::string getModelName();

    std::string getDeviceName();

private:
    std::string manufacture_name;

    std::string model_name;

    std::string device_name;
};

#endif // GVDEVICE_H

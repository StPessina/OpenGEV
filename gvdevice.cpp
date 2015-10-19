#include "gvdevice.h"

GVDevice::GVDevice(std::string manufacture_name, std::string model_name, std::string device_name)
{
    this->manufacture_name = manufacture_name;
    this->model_name = model_name;
    this->device_name = device_name;
}

std::string GVDevice::getManufactureName()
{
    return manufacture_name;
}

std::string GVDevice::getModelName()
{
    return model_name;
}

std::string GVDevice::getDeviceName()
{
    return device_name;
}

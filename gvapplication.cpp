#include "gvapplication.h"

GVApplication::GVApplication()
{
}

GVApplication::~GVApplication()
{

}

void GVApplication::addDevice(Device aDevice)
{
    devices.push_back(aDevice);
}

QList<Device> GVApplication::getDiscoveredDevice()
{
    return devices;
}

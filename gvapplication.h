#ifndef GVAPPLICATION_H
#define GVAPPLICATION_H

#include "gvcomponent.h"

#include <string>
#include <QHostAddress>

using namespace std;
struct Device {
    QString macAddress;
    QHostAddress ipAddress;
    QHostAddress subnetMask;
    QHostAddress defaultGateway;

    QString manufactureName;
    QString modelName;
    QString deviceVersion;

};

class GVApplication : public GVComponent
{
public:
    GVApplication();
    virtual ~GVApplication();

    void addDevice(Device aDevice);

    QList<Device> getDiscoveredDevice();

private:

    QList<Device> devices;
};

#endif // GVAPPLICATION_H

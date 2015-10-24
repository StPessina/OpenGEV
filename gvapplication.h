#ifndef GVAPPLICATION_H
#define GVAPPLICATION_H

#include <QList>

#include "gvcomponent.h"

#include "controlchannelmaster.h"

#include "partnerdevice.h"

#include "discoverycommand.h"

class GVApplication : public GVComponent
{
public:
    GVApplication(int primaryChannelport=5000);
    virtual ~GVApplication();

    void clearDevices();

    void addDevice(PartnerDevice aDevice);

    QList<PartnerDevice> getDiscoveredDevice();


    int discoverDevice();

private:

    QList<PartnerDevice> devices;

    ControlChannelMaster* masterChannel;

};

#endif // GVAPPLICATION_H

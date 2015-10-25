#ifndef GVAPPLICATION_H
#define GVAPPLICATION_H

#include <QList>

#include "CommonComponent/gvcomponent.h"

#include "CommonControlChannel/controlchannelmaster.h"

#include "partnerdevice.h"

#include "ApplicationCommand/discoverycommand.h"

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

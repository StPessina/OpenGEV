#include "gvapplication.h"

GVApplication::GVApplication(int primaryChannelport)
{
    masterChannel = new ControlChannelMaster(QHostAddress::Any, primaryChannelport);
    masterChannel->initSocket();

}

GVApplication::~GVApplication()
{
    delete masterChannel;
}

void GVApplication::clearDevices()
{
    devices.clear();
}

void GVApplication::addDevice(PartnerDevice aDevice)
{
    devices.push_back(aDevice);
}

QList<PartnerDevice> GVApplication::getDiscoveredDevice()
{
    return devices;
}

int GVApplication::discoverDevice()
{
    DiscoveryCommand* dis = new DiscoveryCommand(this);
    int result = masterChannel->sendCommand(dis);
    delete dis;
    return result;
}

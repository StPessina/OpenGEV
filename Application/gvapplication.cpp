#include "gvapplication.h"

GVApplication::GVApplication(int primaryChannelport)
{
    masterChannel = new UDPChannelTransmitter(QHostAddress::Any, primaryChannelport);
    masterChannel->initSocket();

}

GVApplication::~GVApplication()
{
    devices.clear();

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
    QHostAddress address("255.255.255.255");
    DiscoveryCommand dis (this, address, CONTROL_CHANNEL_DEF_PORT);
    int result = masterChannel->sendCommand(dis);
    return result;
}



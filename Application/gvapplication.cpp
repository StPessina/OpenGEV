#include "gvapplication.h"

GVApplication::GVApplication(int primaryChannelport)
{
#ifdef USE_QT_SOCKET
    masterChannel = new QtUDPChannel(QHostAddress::Any, primaryChannelport);
#endif
#ifdef USE_BOOST_SOCKET
    masterChannel = new BoostUDPChannel(QHostAddress::Any, primaryChannelport);
#endif
#ifdef USE_OSAPI_SOCKET
    masterChannel = new OSAPIUDPChannel(QHostAddress::Any, primaryChannelport);
#endif
    masterChannel->initSocket();
    masterChannel->start();


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
    masterChannel->sendPacket(dis);
    return devices.size();
}



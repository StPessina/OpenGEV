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

    masterChannel->quit();
    masterChannel->wait();
    delete masterChannel;
}

void GVApplication::clearDevices()
{
    foreach (PartnerDevice* aDevice, devices)
        delete aDevice;
    devices.clear();
}

void GVApplication::addDevice(PartnerDevice *aDevice)
{
    devices.push_back(aDevice);
}

const QList<PartnerDevice *> GVApplication::getDiscoveredDevices()
{
    return devices;
}

int GVApplication::discoverDevices()
{
    QHostAddress address("255.255.255.255");
    DiscoveryCommand dis (this, address, CONTROL_CHANNEL_DEF_PORT);
    masterChannel->sendPacket(dis);
    return devices.size();
}



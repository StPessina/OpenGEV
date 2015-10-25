#ifndef PARTNERDEVICE_H
#define PARTNERDEVICE_H

#include <QString>
#include <QHostAddress>

#include <unordered_map>

#include "CommonControlChannel/controlchannelmaster.h"

class PartnerDevice
{
public:
    PartnerDevice();

    QString macAddress;
    QHostAddress ipAddress;
    QHostAddress subnetMask;
    QHostAddress defaultGateway;

    QString manufactureName;
    QString modelName;
    QString deviceVersion;

private:

    std::unordered_map<int, ControlChannelMaster*> controlChannels;

};

#endif // PARTNERDEVICE_H

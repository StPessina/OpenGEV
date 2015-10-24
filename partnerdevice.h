#ifndef PARTNERDEVICE_H
#define PARTNERDEVICE_H

#include <QString>
#include <QHostAddress>

#include "controlchannelmaster.h"

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

    ControlChannelMaster* controlChannel;

};

#endif // PARTNERDEVICE_H

#ifndef PARTNERDEVICE_H
#define PARTNERDEVICE_H

#include <QString>
#include <QHostAddress>

#include <unordered_map>

#include "CommonControlChannel/controlchannelmaster.h"

/**
 * @brief The PartnerDevice class provide method for a discovered device
 */
class PartnerDevice
{
public:
    /**
     * @brief PartnerDevice constructor
     */
    PartnerDevice();

    /**
     * @brief ~PartnerDevice object deconstructor
     */
    virtual ~PartnerDevice();

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

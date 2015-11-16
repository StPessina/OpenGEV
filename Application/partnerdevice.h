#ifndef PARTNERDEVICE_H
#define PARTNERDEVICE_H

#include <QString>
#include <QHostAddress>

#include "opengv.h"

#include "CommonComponent/gvcomponent.h"

#include "CommonControlChannel/controlchannelmaster.h"

#include "Device/deviceregisters.h"

#include "ApplicationCommand/readregistercommand.h"
#include "ApplicationCommand/writeregistercommand.h"

/**
 * @brief The PartnerDevice class provide method for a discovered device
 */
class PartnerDevice : public GVComponent
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

    /**
     * @brief openControlChannel send message for channel open on device
     * @param port primary port on the application
     * @return true if the channel is open
     */
    bool openControlChannel(quint16 port);

    /**
     * @brief closeControlChannel send message for close the channel(if it's open)
     */
    void closeControlChannel();

    /**
     * @brief isChannelOpen
     * @return true if the channel is open
     */
    bool isChannelOpen();

    /**
     * @brief getStreamingChannelNumber
     * @param device
     * @return
     */
    int getStreamingChannelNumber();

    /**
     * @brief setControlAccessKey
     * @param device
     * @param key
     */
    bool setControlAccessKey(int key);

private:

    ControlChannelMaster* controlChannel;

    bool channelOpen;

};

#endif // PARTNERDEVICE_H

#ifndef PARTNERDEVICE_H
#define PARTNERDEVICE_H

#include <math.h>

#include <QString>
#include <QHostAddress>
#include <QThread>
#include <unordered_map>

#include "opengv.h"

#include "CommonComponent/gvcomponent.h"

#include "CommonUdpChannel/udpchanneltransmitter.h"

#include "Device/deviceregisters.h"

#include "ApplicationCommand/readregistercommand.h"
#include "ApplicationCommand/writeregistercommand.h"

#include "Application/streamdatareceiver.h"

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
     * @return stream channel number or -1 if the channel is not open
     */
    int getStreamingChannelNumber();

    /**
     * @brief openStreamChannel
     * @param channel
     * @return status result
     */
    int openStreamChannel(int channel);


    StreamDataReceiver *getStreamChannel(int channel);

    /**
     * @brief setControlAccessKey
     * @param device
     * @param key
     * @return false if the channel is not open
     */
    bool setActionControlAccessKey(int key);

    bool setStreamChannelDelay(int channel, quint32 delay);

    bool setStreamChannelPacketLength(int channel, quint32 size);

private:

    UDPChannelTransmitter* controlChannel;

    std::unordered_map<int, StreamDataReceiver*> streamChannelsOpenMap;

    bool channelOpen;

    short controlChannelKey;

};

#endif // PARTNERDEVICE_H

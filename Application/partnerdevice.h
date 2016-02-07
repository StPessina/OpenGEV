#ifndef PARTNERDEVICE_H
#define PARTNERDEVICE_H

#include <math.h>

#include <QString>
#include <QHostAddress>
#include <QThread>
#include <unordered_map>

#include "opengev.h"

#include "CommonComponent/gvcomponent.h"

#ifdef USE_QT_SOCKET
    #include "CommonUdpChannel/qtudpchannel.h"
#endif
#ifdef USE_BOOST_SOCKET
    #include "CommonUdpChannel/boostudpchannel.h"
#endif
#ifdef USE_OSAPI_SOCKET
    #include "CommonUdpChannel/osapiudpchannel.h"
#endif

#include "Device/deviceregisters.h"

#include "ApplicationCommand/readregistercommand.h"
#include "ApplicationCommand/writeregistercommand.h"

#include "Application/streamdatareceiver.h"

/**
 * @brief The PartnerDevice class provide method for a discovered device
 * Through this class application can create a control channel and open stream
 * channel for data receiving.
 * It store stream channels receiver associated to a device.
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

    /**
     * @brief device macAddress
     */
    QString macAddress;

    /**
     * @brief device ipAddress
     */
    QHostAddress ipAddress;

    /**
     * @brief device subnetMask
     */
    QHostAddress subnetMask;

    /**
     * @brief device defaultGateway
     */
    QHostAddress defaultGateway;

    /**
     * @brief device manufactureName
     */
    QString manufactureName;

    /**
     * @brief device modelName
     */
    QString modelName;

    /**
     * @brief device deviceVersion
     */
    QString deviceVersion;

    /**
     * @brief openControlChannel send message for channel open on device
     * @param port used to open and send command to the device
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

    /**
     * @brief getStreamChannel return reference to a open stream channel
     * @param channel number
     * @return stream channel or null if channel number is not valid
     */
    StreamDataReceiver *getStreamChannel(int channel);

    /**
     * @brief closeStreamChannel
     * @param channel
     * @return status result
     */
    int closeStreamChannel(int channel);

    /**
     * @brief setControlAccessKey set access key for switch over mode
     * application can set access key on the device only if application has take
     * control of the device
     * @param key new key value
     * @return false if the channel is not open
     */
    bool setActionControlAccessKey(int key);

    /**
     * @brief setStreamChannelDelay method set delay between payload packet
     * trasmission on the stream channel. Application must has control on the
     * device to set delay
     * @param channel channel number
     * @param delay in nanoseconds
     * @return true if delay is successful set
     */
    bool setStreamChannelDelay(int channel, quint32 delay);

    /**
     * @brief setStreamChannelPacketLength method set packet size of each payload packet
     * on the stream channel. Application must has control on the device to set packet size
     * @param channel channel number
     * @param size packet size in bytes
     * @return true if packet size is successful set
     */
    bool setStreamChannelPacketLength(int channel, quint32 size);

    /**
     * @brief getStreamChannelPacketLength method get packet size of each payload packet
     * on the stream channel.
     * @param channel channel number
     * @return the payload size value or 0 if request fails or -1 if channel is not open
     */
    quint32 getStreamChannelPacketLength(int channel);

    /**
     * Check custom 3D capabilities bootstrap register
     *
     * @brief is3DCamera
     * @return true if it is a 3D camera
     */
    bool is3DCamera();

    /**
     * @brief getHorizontalFieldOfView
     * @return horizontal field of view, or 0 if it's not set
     */
    quint32 getHorizontalFieldOfView();


    /**
     * @brief getverticalFieldOfView
     * @return vertical field of view, or 0 if it's not set
     */
    quint32 getVerticalFieldOfView();

private:

    /**
     * @brief controlChannel member store control channel for
     * command send
     */
    UDPChannel* controlChannel;

    /**
     * @brief streamChannelsOpenMap store opened stream channels
     */
    std::unordered_map<int, StreamDataReceiver*> streamChannelsOpenMap;

    /**
     * @brief channelOpen is true if control channel is open
     */
    bool channelOpen;

    /**
     * @brief controlChannelKey is the last set control channel key
     */
    short controlChannelKey;

};

#endif // PARTNERDEVICE_H

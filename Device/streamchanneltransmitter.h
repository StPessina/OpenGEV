#ifndef DEVICESTREAMCHANNEL_H
#define DEVICESTREAMCHANNEL_H

#include <QObject>

#include <map>
#include <math.h>

#include "opengv_global.h"

#include "CommonBootstrapRegister/bootstrapregister.h"

#ifdef USE_QT_SOCKET
    #include "CommonUdpChannel/qtudpchannel.h"
#endif
#ifdef USE_BOOST_SOCKET
    #include "CommonUdpChannel/boostudpchannel.h"
#endif
#ifdef USE_OSAPI_SOCKET
    #include "CommonUdpChannel/osapiudpchannel.h"
#endif

#include "CommonStream/streamimagedataleader.h"
#include "CommonStream/streamimagedatapayload.h"
#include "CommonStream/streamimagedatatrailer.h"
#include "CommonStream/streamimagedataallin.h"

#include "CommonStreamImageFormat/PixelMap.h"

#include "Device/deviceregisters.h"

class StreamChannelTransmitter : public QObject
{
    Q_OBJECT
public:

    /**
     * @brief StreamChannelTransmitter explict constructor for QObject
     * @param parent
     */
    explicit StreamChannelTransmitter(QObject* parent = 0);

    StreamChannelTransmitter(int id);

    StreamChannelTransmitter(int id, int sourcePort);

    ~StreamChannelTransmitter();

    virtual BootstrapRegister *getRegister(int regType) final;

    virtual BootstrapRegister *getRegisterByAbsoluteRegCode(int regCode) final;

    virtual Status setRegister(int registerCode, int value, QHostAddress senderAddr, quint16 senderPort) final;

    virtual void openStreamChannel(QHostAddress destAddress, quint16 port) final;

    virtual void closeStreamChannel() final;

    virtual bool isChannelOpen() final;

    int writeIncomingData(PixelMap<Pixel<2>>::Ptr datapacket);

    int writeIncomingDataAllInFormat(PixelMap<Pixel<2>>::Ptr datapacket);

protected:
    QHostAddress destAddress;

    quint16 destPort;

private:
    int id;

    int channelPortRegCode;
    int packetSizeRegCode;
    int packetDelayRegCode;
    int packetDestinationAddressRegCode;
    int sourcePortRegCode;
    int channelCapabilityRegCode;
    int channelConfigurationRegCode;
    int channelZoneRegCode;
    int channelZoneDirectionRegCode;

    int blockId=1;

    /**
     * @brief map of network register
     */
    map<int,BootstrapRegister*> registers;

    /**
     * @brief initRegisterMap
     */
    void initRegisterMap();

    UDPChannel* streamChannelTransmitter;

#ifdef ENABLE_LOG4CPP
    log4cpp::Category &logger = log4cpp::Category::getInstance("ComponentLog");
#endif

    void setupStardardRegistersValue();

    void initStreamDataDelayTimer();

    QTimer* dataStreamDelay;
    QEventLoop* dataStreamDelayLoop;
};

#endif // DEVICESTREAMCHANNEL_H

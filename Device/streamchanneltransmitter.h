#ifndef DEVICESTREAMCHANNEL_H
#define DEVICESTREAMCHANNEL_H

#include <map>
#include <math.h>


#include "CommonBootstrapRegister/bootstrapregister.h"

#include "CommonUdpChannel/udpchanneltransmitter.h"

#include "CommonStream/streamimagedataleader.h"
#include "CommonStream/streamimagedatapayload.h"
#include "CommonStream/streamimagedatatrailer.h"

#include "CommonStreamImageFormat/PixelMap.h"

#include "Device/deviceregisters.h"

#include <chrono>

class StreamChannelTransmitter
{
public:
    StreamChannelTransmitter(int id);

    StreamChannelTransmitter(int id, int sourcePort);

    ~StreamChannelTransmitter();

    virtual BootstrapRegister *getRegister(int regType) final;

    virtual BootstrapRegister *getRegisterByAbsoluteRegCode(int regCode) final;

    virtual Status setRegister(int registerCode, int value, QHostAddress senderAddr, quint16 senderPort) final;

    virtual void openStreamChannel(QHostAddress destAddress, quint16 port) final;

    virtual void closeStreamChannel() final;

    virtual bool isChannelOpen() final;

    int writeIncomingData(PixelMap<Pixel>::Ptr datapacket);

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

    UDPChannelTransmitter* streamChannelTransmitter;

    log4cpp::Category &logger = log4cpp::Category::getInstance("ComponentLog");

    void setupStardardRegistersValue();


};

#endif // DEVICESTREAMCHANNEL_H

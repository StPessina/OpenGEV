#ifndef DEVICESTREAMCHANNEL_H
#define DEVICESTREAMCHANNEL_H

#include <map>
#include <math.h>


#include "CommonBootstrapRegister/bootstrapregister.h"

#include "CommonUdpChannel/udpchanneltransmitter.h"

#include "Device/deviceregisters.h"

class StreamChannelTransmitter
{
public:
    StreamChannelTransmitter(int id);

    StreamChannelTransmitter(int id, int sourcePort);

    ~StreamChannelTransmitter();

    virtual BootstrapRegister *getRegister(int regType) final;

    virtual BootstrapRegister *getRegisterByAbsoluteRegCode(int regCode) final;

    virtual void openStreamChannel(QHostAddress destAddress, quint16 port) final;

    virtual void closeStreamChannel() final;

    virtual bool isChannelOpen() final;

    virtual void writeIncomingData(AbstractCommand* datapacket) = 0;

protected:
    QHostAddress destAddress;

    quint16 destPort;

private:
    int id;

    /**
     * @brief map of network register
     */
    map<int,BootstrapRegister*> registers;

    /**
     * @brief initRegisterMap
     */
    void initRegisterMap();

    UDPChannelTransmitter* streamChannelTransmitter;


};

#endif // DEVICESTREAMCHANNEL_H

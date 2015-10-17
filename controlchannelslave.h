#ifndef DEVICECONTROLCHANNEL_H
#define DEVICECONTROLCHANNEL_H

#include "controlchannel.h"
#include "devicemessagehandlerfactory.h"

class ControlChannelSlave : public ControlChannel
{
public:
    ControlChannelSlave(QHostAddress sourceAddr,
                         quint16 sourcePort,
                         AbstractMessageHandlerFactory *messageHandlerFactory);

    void processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort);
};

#endif // DEVICECONTROLCHANNEL_H

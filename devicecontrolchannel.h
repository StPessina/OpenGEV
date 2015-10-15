#ifndef DEVICECONTROLCHANNEL_H
#define DEVICECONTROLCHANNEL_H

#include "controlchannel.h"
#include "devicemessagefactory.h"

class DeviceControlChannel : public ControlChannel
{
public:
    DeviceControlChannel(QHostAddress sourceAddr,
                         quint16 sourcePort,
                         DeviceMessageFactory *messageHandlerFactory);

    void processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort);
};

#endif // DEVICECONTROLCHANNEL_H

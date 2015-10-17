#ifndef APPLICATIONCONTROLCHANNEL_H
#define APPLICATIONCONTROLCHANNEL_H

#include "controlchannel.h"

#include "applicationmessagehandlerfactory.h"

#include "abstractcommand.h"

class ControlChannelMaster : public ControlChannel
{
public:
    ControlChannelMaster(QHostAddress sourceAddr,
                              quint16 sourcePort,
                              AbstractMessageHandlerFactory *messageHandlerFactory);

    void processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort);

    void sendCommand(AbstractCommand* cmd);

private:
    static int timeout;

    bool waitForAck;
};

#endif // APPLICATIONCONTROLCHANNEL_H

#ifndef APPLICATIONCONTROLCHANNEL_H
#define APPLICATIONCONTROLCHANNEL_H

#include "controlchannel.h"

#include "applicationmessagefactory.h"

#include "abstractcommand.h"

class ApplicationControlChannel : public ControlChannel
{
public:
    ApplicationControlChannel(QHostAddress sourceAddr,
                              quint16 sourcePort,
                              ApplicationMessageFactory* messageHandlerFactory);

    void processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort);

    void sendCommand(AbstractCommand* cmd);
};

#endif // APPLICATIONCONTROLCHANNEL_H

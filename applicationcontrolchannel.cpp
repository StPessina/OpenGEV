#include "applicationcontrolchannel.h"

ApplicationControlChannel::ApplicationControlChannel(QHostAddress sourceAddr,
                                                     quint16 sourcePort,
                                                     ApplicationMessageFactory* messageHandlerFactory)
    : ControlChannel(sourceAddr, sourcePort, messageHandlerFactory)
{
}

void ApplicationControlChannel::processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort)
{

}

void ApplicationControlChannel::sendCommand(AbstractCommand *cmd)
{

}


#include "discoverymessagehandler.h"

DiscoveryMessageHandler::DiscoveryMessageHandler(GVDevice* target, QByteArray datagram,
                                                 QHostAddress senderAddress, quint16 senderPort)
    : AbstractMessageHandler(target, DISCOVERY_ACK, datagram, senderAddress, senderPort)
{
}

bool DiscoveryMessageHandler::isAllowed(Privilege ctrlChannelPrivilege)
{
    return true;
}

int DiscoveryMessageHandler::execute(Privilege ctrlChannelPrivilege)
{
    return 0;
}

int DiscoveryMessageHandler::getAckDatagramLengthWithoutHeader()
{
    return 0;
}

char *DiscoveryMessageHandler::getAckDatagramWithoutHeader()
{
    return new char[1];
}



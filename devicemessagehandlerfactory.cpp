#include "devicemessagehandlerfactory.h"

DeviceMessageHandlerFactory::DeviceMessageHandlerFactory(GVDevice *target)
    : AbstractMessageHandlerFactory(target)
{
}

bool DeviceMessageHandlerFactory::isValidCode(int messageCode)
{
    return true;
}

AbstractMessageHandler *DeviceMessageHandlerFactory::createMessageHandler(int messageCode, QByteArray datagram,
                                                                   QHostAddress senderAddress, quint16 senderPort)
{
    switch (messageCode) {
    case DISCOVERY_CMD:
        return new DiscoveryMessageHandler((GVDevice*)target, datagram, senderAddress, senderPort);
    default:
        return new CmdNotSupportedMH(target, messageCode, datagram, senderAddress, senderPort);
    }
}

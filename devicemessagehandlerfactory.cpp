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
    return new DiscoveryMessageHandler(new GVDevice, datagram, senderAddress, senderPort);
}

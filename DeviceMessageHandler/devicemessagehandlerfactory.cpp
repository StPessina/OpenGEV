#include "devicemessagehandlerfactory.h"

DeviceMessageHandlerFactory::DeviceMessageHandlerFactory(GVDevice *target)
    : AbstractMessageHandlerFactory(dynamic_cast<GVComponent*>(target))
{
}

bool DeviceMessageHandlerFactory::isValidCode(quint16 messageCode)
{
    switch (messageCode) {
    case DISCOVERY_CMD:
    case READREG_CMD:
    case WRITEREG_CMD:
        return true;
    default:
        break;
    }
    return false;
}

AbstractMessageHandler *DeviceMessageHandlerFactory::createMessageHandler(quint16 messageCode, QByteArray datagram,
                                                                   QHostAddress senderAddress, quint16 senderPort)
{
    switch (messageCode) {
    case DISCOVERY_CMD:
        return new DiscoveryMessageHandler(dynamic_cast<GVDevice*>(target), datagram, senderAddress, senderPort);
    case READREG_CMD:
        return new ReadRegisterMessageHandler(dynamic_cast<GVDevice*>(target), datagram, senderAddress, senderPort);
    case WRITEREG_CMD:
        return new WriteRegisterMessageHandler(dynamic_cast<GVDevice*>(target), datagram, senderAddress, senderPort);
    default:
        return new CmdNotSupportedMH(target, messageCode, datagram, senderAddress, senderPort);
    }
}

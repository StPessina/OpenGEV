#include "devicecommandhandlerfactory.h"

DeviceCommandHandlerFactory::DeviceCommandHandlerFactory(GVComponent * const target)
    : AbstractCommandHandlerFactory(target)
{
}

bool DeviceCommandHandlerFactory::isValidCode(quint16 handlerIdentifier)
{
    switch (handlerIdentifier) {
    case DISCOVERY_CMD:
    case READREG_CMD:
    case WRITEREG_CMD:
        return true;
    default:
        break;
    }
    return false;
}

AbstractCommandHandler *DeviceCommandHandlerFactory::createPacketHandler(quint16 handlerIdentifier, const QByteArray &datagram,
                                                                   QHostAddress senderAddress, quint16 senderPort)
{
    switch (handlerIdentifier) {
    case DISCOVERY_CMD:
        return new DiscoveryCommandHandler(dynamic_cast<GVDevice*>(target), datagram, senderAddress, senderPort);
    case READREG_CMD:
        return new ReadRegisterCommandHandler(dynamic_cast<GVDevice*>(target), datagram, senderAddress, senderPort);
    case WRITEREG_CMD:
        return new WriteRegisterCommandHandler(dynamic_cast<GVDevice*>(target), datagram, senderAddress, senderPort);
    default:
        return new CmdNotSupportedMH(target, handlerIdentifier, datagram, senderAddress, senderPort);
    }
}

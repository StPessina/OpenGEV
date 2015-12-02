#include "applicationcommandhandlerfactory.h"

ApplicationCommandHandlerFactory::ApplicationCommandHandlerFactory(GVApplication* target)
    : AbstractCommandHandlerFactory(dynamic_cast<GVComponent*>(target))
{
}

ApplicationCommandHandlerFactory::~ApplicationCommandHandlerFactory()
{

}

bool ApplicationCommandHandlerFactory::isValidCode(quint16 handlerIdentifier)
{
    return true;
}

AbstractCommandHandler *ApplicationCommandHandlerFactory::createPacketHandler(quint16 handlerIdentifier,
                                                                               const QByteArray &datagram,
                                                                               QHostAddress senderAddress,
                                                                               quint16 senderPort)
{
    return new CmdNotSupportedMH(target,handlerIdentifier,datagram,senderAddress, senderPort);
}

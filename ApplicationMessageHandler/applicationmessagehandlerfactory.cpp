#include "applicationmessagehandlerfactory.h"

ApplicationMessageHandlerFactory::ApplicationMessageHandlerFactory(GVApplication* target)
    : AbstractMessageHandlerFactory(dynamic_cast<GVComponent*>(target))
{
}

ApplicationMessageHandlerFactory::~ApplicationMessageHandlerFactory()
{

}

bool ApplicationMessageHandlerFactory::isValidCode(quint16 messageCode)
{
    return true;
}

AbstractMessageHandler *ApplicationMessageHandlerFactory::createMessageHandler(quint16 messageCode,
                                                                               QByteArray datagram,
                                                                               QHostAddress senderAddress,
                                                                               quint16 senderPort)
{
    return new CmdNotSupportedMH(target,messageCode,datagram,senderAddress, senderPort);
}

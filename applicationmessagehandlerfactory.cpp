#include "applicationmessagehandlerfactory.h"

ApplicationMessageHandlerFactory::ApplicationMessageHandlerFactory(GVApplication* target)
    : AbstractMessageHandlerFactory(target)
{
}

ApplicationMessageHandlerFactory::~ApplicationMessageHandlerFactory()
{

}

bool ApplicationMessageHandlerFactory::isValidCode(int messageCode)
{
    return true;
}

AbstractMessageHandler *ApplicationMessageHandlerFactory::createMessageHandler(int messageCode,
                                                                               QByteArray datagram,
                                                                               QHostAddress senderAddress,
                                                                               quint16 senderPort)
{
    return new CmdNotSupportedMH(target,messageCode,datagram,senderAddress, senderPort);
}

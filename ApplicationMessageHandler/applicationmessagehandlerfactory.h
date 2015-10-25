#ifndef APPLICATIONMESSAGEHANDLERFACTORY_H
#define APPLICATIONMESSAGEHANDLERFACTORY_H

#include "CommonMessages/abstractmessagehandlerfactory.h"

#include "Application/gvapplication.h"

#include "DeviceMessageHandler/cmdnotsupportedmh.h"

class ApplicationMessageHandlerFactory : public AbstractMessageHandlerFactory
{
public:
    ApplicationMessageHandlerFactory(GVApplication* target);
    virtual ~ApplicationMessageHandlerFactory();

    bool isValidCode(int messageCode);

    AbstractMessageHandler *createMessageHandler(int messageCode,
                                                 QByteArray datagram,
                                                 QHostAddress senderAddress,
                                                 quint16 senderPort);
};

#endif // APPLICATIONMESSAGEHANDLERFACTORY_H

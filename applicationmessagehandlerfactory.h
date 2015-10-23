#ifndef APPLICATIONMESSAGEHANDLERFACTORY_H
#define APPLICATIONMESSAGEHANDLERFACTORY_H

#include "abstractmessagehandlerfactory.h"

#include "gvapplication.h"

#include "cmdnotsupportedmh.h"

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

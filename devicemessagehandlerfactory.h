#ifndef DEVICEMESSAGEHANDLERFACTORY_H
#define DEVICEMESSAGEHANDLERFACTORY_H

#include "abstractmessagehandlerfactory.h"

#include "gvdevice.h"

#include "ApplicationCommandCode.h"

#include "cmdnotsupportedmh.h"

#include "discoverymessagehandler.h"

class DeviceMessageHandlerFactory : public AbstractMessageHandlerFactory
{
public:
    DeviceMessageHandlerFactory(GVDevice* target);

    bool isValidCode(int messageCode);

    AbstractMessageHandler* createMessageHandler(int messageCode,
                                                 QByteArray datagram,
                                                 QHostAddress senderAddress,
                                                 quint16 senderPort);
};

#endif // DEVICEMESSAGEHANDLERFACTORY_H

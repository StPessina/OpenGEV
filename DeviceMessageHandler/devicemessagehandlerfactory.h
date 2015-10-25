#ifndef DEVICEMESSAGEHANDLERFACTORY_H
#define DEVICEMESSAGEHANDLERFACTORY_H

#include "CommonMessages/abstractmessagehandlerfactory.h"

#include "Device/gvdevice.h"

#include "ApplicationCommand/applicationcommandcode.h"

#include "DeviceMessageHandler/cmdnotsupportedmh.h"
#include "DeviceMessageHandler/discoverymessagehandler.h"

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

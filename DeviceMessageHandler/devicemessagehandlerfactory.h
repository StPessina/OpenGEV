#ifndef DEVICEMESSAGEHANDLERFACTORY_H
#define DEVICEMESSAGEHANDLERFACTORY_H

#include "CommonMessages/abstractmessagehandlerfactory.h"

#include "Device/gvdevice.h"

#include "ApplicationCommand/applicationcommandcode.h"

#include "DeviceMessageHandler/cmdnotsupportedmh.h"
#include "DeviceMessageHandler/discoverymessagehandler.h"
#include "DeviceMessageHandler/readregistermessagehandler.h"

/**
 * @brief The DeviceMessageHandlerFactory class is handler for device message
 */
class DeviceMessageHandlerFactory : public AbstractMessageHandlerFactory
{
public:
    DeviceMessageHandlerFactory(GVDevice* target);

    bool isValidCode(quint16 messageCode);

    AbstractMessageHandler* createMessageHandler(quint16 messageCode,
                                                 QByteArray datagram,
                                                 QHostAddress senderAddress,
                                                 quint16 senderPort);
};

#endif // DEVICEMESSAGEHANDLERFACTORY_H

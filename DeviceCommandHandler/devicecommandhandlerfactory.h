#ifndef DEVICECOMMANDHANDLERFACTORY_H
#define DEVICECOMMANDHANDLERFACTORY_H

#include "CommonCommand/abstractcommandhandlerfactory.h"

#include "Device/gvdevice.h"

#include "ApplicationCommand/applicationcommandcode.h"

#include "DeviceCommandHandler/cmdnotsupportedmh.h"
#include "DeviceCommandHandler/discoverycommandhandler.h"
#include "DeviceCommandHandler/readregistercommandhandler.h"
#include "DeviceCommandHandler/writeregistercommandhandler.h"

/**
 * @brief The DeviceMessageHandlerFactory class is handler for device message
 */
class DeviceCommandHandlerFactory : public AbstractCommandHandlerFactory
{
public:
    DeviceCommandHandlerFactory(GVDevice* target);

    bool isValidCode(quint16 messageCode);

    AbstractCommandHandler* createPacketHandler(quint16 messageCode,
                                                 QByteArray datagram,
                                                 QHostAddress senderAddress,
                                                 quint16 senderPort);
};

#endif // DEVICECOMMANDHANDLERFACTORY_H

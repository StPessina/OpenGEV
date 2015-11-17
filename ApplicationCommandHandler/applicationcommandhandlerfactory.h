#ifndef APPLICATIONCOMMANDHANDLERFACTORY_H
#define APPLICATIONCOMMANDHANDLERFACTORY_H

#include "CommonCommand/abstractcommandhandlerfactory.h"

#include "Application/gvapplication.h"

#include "DeviceCommandHandler/cmdnotsupportedmh.h"

/**
 * @brief The ApplicationMessageHandlerFactory class provide factory pattern for devices, based on the command
 * request id
 */
class ApplicationCommandHandlerFactory : public AbstractCommandHandlerFactory
{
public:
    /**
     * @brief ApplicationMessageHandlerFactory constructor
     * @param target is the application where the commands handled will be executed
     */
    ApplicationCommandHandlerFactory(GVApplication* target);

    /**
     * @brief ~ApplicationMessageHandlerFactory deconstructor
     */
    virtual ~ApplicationCommandHandlerFactory();

    /**
     * @brief isValidCode method check if a messageCode has a specific messageHandler
     * @param messageCode
     * @return true if a message handler exists with the requested messageCode
     */
    bool isValidCode(quint16 handlerIdentifier);

    /**
     * @brief createMessageHandler method create a message handler for a message
     * @param messageCode the code of the message to manage
     * @param datagram the datagram of the command
     * @param senderAddress
     * @param senderPort
     * @return a message handler if exist, or generic message non supported handler
     */
    AbstractCommandHandler *createPacketHandler(quint16 handlerIdentifier,
                                                 QByteArray datagram,
                                                 QHostAddress senderAddress,
                                                 quint16 senderPort);
};

#endif // APPLICATIONCOMMANDHANDLERFACTORY_H

#ifndef APPLICATIONMESSAGEHANDLERFACTORY_H
#define APPLICATIONMESSAGEHANDLERFACTORY_H

#include "CommonMessages/abstractmessagehandlerfactory.h"

#include "Application/gvapplication.h"

#include "DeviceMessageHandler/cmdnotsupportedmh.h"

/**
 * @brief The ApplicationMessageHandlerFactory class provide factory pattern for devices, based on the command
 * request id
 */
class ApplicationMessageHandlerFactory : public AbstractMessageHandlerFactory
{
public:
    /**
     * @brief ApplicationMessageHandlerFactory constructor
     * @param target is the application where the commands handled will be executed
     */
    ApplicationMessageHandlerFactory(GVApplication* target);

    /**
     * @brief ~ApplicationMessageHandlerFactory deconstructor
     */
    virtual ~ApplicationMessageHandlerFactory();

    /**
     * @brief isValidCode method check if a messageCode has a specific messageHandler
     * @param messageCode
     * @return true if a message handler exists with the requested messageCode
     */
    bool isValidCode(quint16 messageCode);

    /**
     * @brief createMessageHandler method create a message handler for a message
     * @param messageCode the code of the message to manage
     * @param datagram the datagram of the command
     * @param senderAddress
     * @param senderPort
     * @return a message handler if exist, or generic message non supported handler
     */
    AbstractMessageHandler *createMessageHandler(quint16 messageCode,
                                                 QByteArray datagram,
                                                 QHostAddress senderAddress,
                                                 quint16 senderPort);
};

#endif // APPLICATIONMESSAGEHANDLERFACTORY_H

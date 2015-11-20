#ifndef APPLICATIONCOMMANDHANDLERFACTORY_H
#define APPLICATIONCOMMANDHANDLERFACTORY_H

#include "CommonCommand/abstractcommandhandlerfactory.h"

#include "CommonComponent/gvcomponent.h"

#include "ApplicationStreamDataHandler/streamimagedataleaderhandler.h"
#include "ApplicationStreamDataHandler/streamimagedatapayloadhandler.h"
#include "ApplicationStreamDataHandler/streamimagedatatrailerhandler.h"

#include "DeviceCommandHandler/cmdnotsupportedmh.h"

/**
 * @brief The ApplicationMessageHandlerFactory class provide factory pattern for devices, based on the command
 * request id
 */
class StreamImageDataHandlerFactory : public AbstractPacketHandlerFactory
{
public:
    /**
     * @brief ApplicationMessageHandlerFactory constructor
     * @param target is the application where the commands handled will be executed
     */
    StreamImageDataHandlerFactory(GVComponent* const target);

    /**
     * @brief ~ApplicationMessageHandlerFactory deconstructor
     */
    virtual ~StreamImageDataHandlerFactory();

    int getPacketHandlerIdentifier(QByteArray datagram) final;

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
    AbstractStreamDataHandler *createPacketHandler(quint16 handlerIdentifier,
                                                 QByteArray datagram,
                                                 QHostAddress senderAddress,
                                                 quint16 senderPort);
};

#endif // APPLICATIONCOMMANDHANDLERFACTORY_H

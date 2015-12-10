#ifndef ABSTRACTCOMMANDHANDLERFACTORY_H
#define ABSTRACTCOMMANDHANDLERFACTORY_H

#include "CommonComponent/gvcomponent.h"

#include "CommonPacket/abstractpackethandlerfactory.h"
#include "CommonCommand/abstractcommandhandler.h"

#include "CommonPacket/conversionutils.h"

/**
 * @brief The AbstractCommandHandlerFactory class create new message handlers for commands
 * messages.
 */
class AbstractCommandHandlerFactory : public AbstractPacketHandlerFactory
{
public:
    /**
     * @brief AbstractCommandHandlerFactory constructor
     * @param target the component where the handler created will be executed
     */
    AbstractCommandHandlerFactory(GVComponent* const target);

    /**
     * @brief ~AbstractMessageHandlerFactory deconstructor
     */
    virtual ~AbstractCommandHandlerFactory();

    /**
     * @brief getPacketHandlerIdentifier search handler identifier inside datagrams
     * @param datagram
     * @return handler identief if found, 0 if not found, -1 if problems with header
     */
    int getPacketHandlerIdentifier(const QByteArray &datagram) final;

    /**
     * @brief isValidCode check if a message code exist
     * @param messageCode
     * @return true if the code is valid and an handler exist
     */
    virtual bool isValidCode(quint16 handlerIdentifier) = 0;

    /**
     * @brief createMessageHandler
     * @param messageCode
     * @param datagram
     * @param senderAddress
     * @param senderPort
     * @return message handler for the requested message code
     */
    virtual AbstractCommandHandler *createPacketHandler(quint16 handlerIdentifier,
                                                         const QByteArray &datagram,
                                                         QHostAddress senderAddress,
                                                         quint16 senderPort) = 0;

protected:
    GVComponent* target;
};

#endif // ABSTRACTCOMMANDHANDLERFACTORY_H

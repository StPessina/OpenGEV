#ifndef ABSTRACTPACKETHANDLERFACTORY_H
#define ABSTRACTPACKETHANDLERFACTORY_H

#include "CommonComponent/gvcomponent.h"
#include "CommonPacket/abstractpackethandler.h"

#include "CommonPacket/conversionutils.h"

/**
 * @brief The AbstractPacketHandlerFactory class create new message handlers
 */
class AbstractPacketHandlerFactory
{
public:
    /**
     * @brief AbstractMessageHandlerFactory constructor
     * @param target the component where the handler created will be executed
     */
    AbstractPacketHandlerFactory(GVComponent* target);

    /**
     * @brief ~AbstractMessageHandlerFactory deconstructor
     */
    virtual ~AbstractPacketHandlerFactory();

    /**
     * @brief getPacketHandlerIdentifier
     * @param datagram
     * @return identifier of packet from datagram
     */
    virtual int getPacketHandlerIdentifier(QByteArray datagram) = 0;

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
    virtual AbstractPacketHandler *createPacketHandler(quint16 handlerIdentifier,
                                                         QByteArray datagram,
                                                         QHostAddress senderAddress,
                                                         quint16 senderPort) = 0;

protected:
    GVComponent* target;
};

#endif // ABSTRACTPACKETHANDLERFACTORY_H

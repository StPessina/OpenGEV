#ifndef ABSTRACTPACKETHANDLERFACTORY_H
#define ABSTRACTPACKETHANDLERFACTORY_H

#include "CommonComponent/gvcomponent.h"
#include "CommonPacket/abstractpackethandler.h"

#include "CommonPacket/conversionutils.h"

/**
 * @brief The AbstractPacketHandlerFactory class create new message handlers. This class can be inherited
 * to create specific message factory. An UDPChannel need a factory to manage incoming messages on the channel.
 */
class AbstractPacketHandlerFactory
{
public:
    /**
     * @brief AbstractPacketHandlerFactory constructor
     * @param target the component where the handler created will be executed
     */
    AbstractPacketHandlerFactory(GVComponent* target);

    /**
     * @brief ~AbstractPacketHandlerFactory deconstructor
     */
    virtual ~AbstractPacketHandlerFactory();

    /**
     * @brief getPacketHandlerIdentifier
     * @param datagram
     * @return identifier of packet from datagram
     */
    virtual int getPacketHandlerIdentifier(const QByteArray &datagram) = 0;

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
                                                         const QByteArray &datagram,
                                                         QHostAddress senderAddress,
                                                         quint16 senderPort) = 0;

protected:
    GVComponent* target;
};

#endif // ABSTRACTPACKETHANDLERFACTORY_H

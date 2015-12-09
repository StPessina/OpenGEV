#ifndef APPLICATIONCOMMANDHANDLERFACTORY_H
#define APPLICATIONCOMMANDHANDLERFACTORY_H

#include <stdio.h>

#include "CommonCommand/abstractcommandhandlerfactory.h"

#include "CommonComponent/gvcomponent.h"

#include "ApplicationStreamDataHandler/streamimagedataleaderhandler.h"
#include "ApplicationStreamDataHandler/streamimagedatapayloadhandler.h"
#include "ApplicationStreamDataHandler/streamimagedatatrailerhandler.h"
#include "ApplicationStreamDataHandler/streamimagedataallinhandler.h"

#include "DeviceCommandHandler/cmdnotsupportedmh.h"

/**
 * @brief The StreamImageDataHandlerFactory class provide factory pattern for stream channel receiver. Each stream
 * packet is handled by specific stream image data handler.
 */
class StreamImageDataHandlerFactory : public AbstractPacketHandlerFactory
{
public:
    /**
     * @brief StreamImageDataHandlerFactory constructor
     * @param target is the stream channel receiver where the stream data is stored
     */
    StreamImageDataHandlerFactory(GVComponent* const target);

    /**
     * @brief ~StreamImageDataHandlerFactory deconstructor
     */
    virtual ~StreamImageDataHandlerFactory();

    /**
     * @brief getPacketHandlerIdentifier
     * @param datagram
     * @return packet handler identifier if exist (handlerIdentifier)
     */
    int getPacketHandlerIdentifier(const QByteArray &datagram) final;

    /**
     * @brief isValidCode method check if a messageCode has a specific streamPacketHandler
     * @param handlerIdentifier
     * @return true if a packet handler exists with the requested handlerIdentifier
     */
    bool isValidCode(quint16 handlerIdentifier);

    /**
     * @brief createMessageHandler method create a message handler for a message
     * @param handlerIdentifier the code of the packet to manage
     * @param datagram the datagram of the packet
     * @param senderAddress
     * @param senderPort
     * @return a packet handler if exist
     */
    AbstractStreamDataHandler *createPacketHandler(quint16 handlerIdentifier,
                                                 const QByteArray &datagram,
                                                 QHostAddress senderAddress,
                                                 quint16 senderPort);
};

#endif // APPLICATIONCOMMANDHANDLERFACTORY_H

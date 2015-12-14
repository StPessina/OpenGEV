#ifndef STREAMIMAGEDATAALLINHANDLER_H
#define STREAMIMAGEDATAALLINHANDLER_H

#include "CommonStream/abstractstreamdatahandler.h"

#include "CommonPacket/conversionutils.h"

#include "CommonStream/packetformat.h"

#include "opengev_global.h"

#include "Application/streamdatareceiver.h"

/**
 * @brief The StreamImageDataAllInHandler class implements stream image data
 * handler for allin packets
 */
class StreamImageDataAllInHandler : public AbstractStreamDataHandler
{
public:

    /**
     * @brief StreamImageDataAllInHandler constructor
     * @param target stream channel receiver where incoming data is stored
     * @param receivedDatagram
     * @param senderAddress
     * @param senderPort
     */
    StreamImageDataAllInHandler(GVComponent* target,
                               const QByteArray &receivedDatagram,
                               QHostAddress senderAddress,
                               quint16 senderPort);

    /**
     * @brief execute copy stream data from datagram to PixelMap
     * @return 0 if executed successful
     */
    int execute();

};

#endif // STREAMIMAGEDATAALLINHANDLER_H

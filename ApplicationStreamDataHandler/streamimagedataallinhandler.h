#ifndef STREAMIMAGEDATAALLINHANDLER_H
#define STREAMIMAGEDATAALLINHANDLER_H

#include "CommonStream/abstractstreamdatahandler.h"

#include "CommonPacket/conversionutils.h"

#include "CommonStream/packetformat.h"

#include "opengv_global.h"

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

protected:

    /**
     * @brief getAckDatagramLengthWithoutHeader
     * @return 0 (R-164c no ack required for stream message
     */
    quint16 getAckDatagramLengthWithoutHeader();

    /**
     * @brief appendAckDatagramWithoutHeader
     * @param datagram (R-164c no ack required for stream message)
     */
    void appendAckDatagramWithoutHeader(QByteArray &datagram);

};

#endif // STREAMIMAGEDATAALLINHANDLER_H

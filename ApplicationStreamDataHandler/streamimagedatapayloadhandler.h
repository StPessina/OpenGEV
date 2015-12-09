#ifndef STREAMIMAGEDATAPAYLOADHANDLER_H
#define STREAMIMAGEDATAPAYLOADHANDLER_H

#include "CommonStream/abstractstreamdatahandler.h"

#include "CommonPacket/conversionutils.h"

#include "CommonStream/packetformat.h"
#include "CommonStreamImageFormat/pixelformat.h"

#include "opengv_global.h"

#include "Application/streamdatareceiver.h"

/**
 * @brief The StreamImageDataPayloadHandler class implements stream image data
 * handler for payload packet
 */
class StreamImageDataPayloadHandler : public AbstractStreamDataHandler
{
public:

    /**
     * @brief StreamImageDataPayloadHandler
     * @param target stream channel receiver where incoming data will be stored
     * @param receivedDatagram
     * @param senderAddress
     * @param senderPort
     */
    StreamImageDataPayloadHandler(GVComponent* target,
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
     * @brief getAckDatagramWithoutHeader
     * @return 0 (R-164c no ack required for stream message)
     */
    quint16 getAckDatagramLengthWithoutHeader();

    /**
     * @brief appendAckDatagramWithoutHeader
     * @param datagram (R-164c no ack required for stream message)
     */
    void appendAckDatagramWithoutHeader(QByteArray &datagram);
};

#endif // STREAMIMAGEDATAPAYLOADHANDLER_H

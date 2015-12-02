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
    StreamImageDataPayloadHandler(GVComponent* target,
                               const QByteArray &receivedDatagram,
                               QHostAddress senderAddress,
                               quint16 senderPort);

    int execute();

protected:

    quint16 getAckDatagramLengthWithoutHeader();

    /**
     * @brief getAckDatagramWithoutHeader
     * @return datagram (R-164c)
     */
    void appendAckDatagramWithoutHeader(QByteArray &datagram);
};

#endif // STREAMIMAGEDATAPAYLOADHANDLER_H

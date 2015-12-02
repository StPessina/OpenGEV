#ifndef STREAMIMAGEDATATRAILERHANDLER_H
#define STREAMIMAGEDATATRAILERHANDLER_H

#include "CommonStream/abstractstreamdatahandler.h"

#include "CommonPacket/conversionutils.h"

#include "CommonStream/packetformat.h"

#include "opengv_global.h"

#include "Application/streamdatareceiver.h"

/**
 * @brief The StreamImageDataTrailerHandler class implements stream image data
 * handler for trailer packet
 */
class StreamImageDataTrailerHandler : public AbstractStreamDataHandler
{
public:
    StreamImageDataTrailerHandler(GVComponent* target,
                               const QByteArray &receivedDatagram,
                               QHostAddress senderAddress,
                               quint16 senderPort);

    int execute();

    quint16 getAckDatagramLengthWithoutHeader();

    /**
     * @brief getAckDatagramWithoutHeader
     * @return datagram (R-164c)
     */
    void appendAckDatagramWithoutHeader(QByteArray &datagram);

};

#endif // STREAMIMAGEDATATRAILERHANDLER_H

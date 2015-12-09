#ifndef STREAMIMAGEDATALEADERHANDLER_H
#define STREAMIMAGEDATALEADERHANDLER_H

#include "CommonStream/abstractstreamdatahandler.h"

#include "CommonPacket/conversionutils.h"

#include "CommonStream/packetformat.h"

#include "opengv_global.h"

#include "Application/streamdatareceiver.h"

/**
 * @brief The StreamImageDataLeaderHandler class implements stream image data
 * handler for leader packet (open PixelData on a stream data receiver)
 */
class StreamImageDataLeaderHandler : public AbstractStreamDataHandler
{
public:

    /**
     * @brief StreamImageDataLeaderHandler constructor
     * @param target stream channel receiver where PixelMap will be opened
     * @param receivedDatagram
     * @param senderAddress
     * @param senderPort
     */
    StreamImageDataLeaderHandler(GVComponent* target,
                               const QByteArray &receivedDatagram,
                               QHostAddress senderAddress,
                               quint16 senderPort);

    /**
     * @brief execute try to open a new PixelMap on the target stream channel receiver
     * to store incoming data
     * @return 0 if executed successful
     */
    int execute();

protected:

    /**
     * @brief getAckDatagramLengthWithoutHeader
     * @return 0 (R-164c no ack required for stream message)
     */
    quint16 getAckDatagramLengthWithoutHeader();

    /**
     * @brief appendAckDatagramWithoutHeader
     * @param datagram (R-164c no ack required for stream message)
     */
    void appendAckDatagramWithoutHeader(QByteArray &datagram);

};

#endif // STREAMIMAGEDATALEADERHANDLER_H

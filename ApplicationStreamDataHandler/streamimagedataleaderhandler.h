#ifndef STREAMIMAGEDATALEADERHANDLER_H
#define STREAMIMAGEDATALEADERHANDLER_H

#include "CommonStream/abstractstreamdatahandler.h"

#include "CommonPacket/conversionutils.h"

#include "CommonStream/packetformat.h"

#include "opengv_global.h"

#include "Application/streamdatareceiver.h"

/**
 * @brief The StreamImageDataLeaderHandler class implements stream image data
 * handler for leader packet
 */
class StreamImageDataLeaderHandler : public AbstractStreamDataHandler
{
public:
    StreamImageDataLeaderHandler(GVComponent* target,
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

#endif // STREAMIMAGEDATALEADERHANDLER_H

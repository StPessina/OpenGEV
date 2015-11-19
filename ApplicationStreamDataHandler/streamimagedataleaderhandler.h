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
                               QByteArray datagram,
                               QHostAddress senderAddress,
                               quint16 senderPort);

    int execute();

    quint16 getAckDatagramLengthWithoutHeader();

    /**
     * @brief getAckDatagramWithoutHeader
     * @return datagram (R-164c)
     */
    char* getAckDatagramWithoutHeader();

};

#endif // STREAMIMAGEDATALEADERHANDLER_H

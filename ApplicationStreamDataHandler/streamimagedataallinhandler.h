#ifndef STREAMIMAGEDATAALLINHANDLER_H
#define STREAMIMAGEDATAALLINHANDLER_H

#include "CommonStream/abstractstreamdatahandler.h"

#include "CommonPacket/conversionutils.h"

#include "CommonStream/packetformat.h"

#include "opengv_global.h"

#include "Application/streamdatareceiver.h"

/**
 * @brief The StreamImageDataAllInHandler class implements stream image data
 * handler for leader packet
 */
class StreamImageDataAllInHandler : public AbstractStreamDataHandler
{
public:
    StreamImageDataAllInHandler(GVComponent* target,
                               QByteArray datagram,
                               QHostAddress senderAddress,
                               quint16 senderPort);

    int execute();

    quint16 getAckDatagramLengthWithoutHeader();

    /**
     * @brief getAckDatagramWithoutHeader
     * @return datagram (R-164c)
     */
    QByteArray getAckDatagramWithoutHeader();

};

#endif // STREAMIMAGEDATAALLINHANDLER_H

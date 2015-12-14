#ifndef STREAMIMAGEDATATRAILERHANDLER_H
#define STREAMIMAGEDATATRAILERHANDLER_H

#include "CommonStream/abstractstreamdatahandler.h"

#include "CommonPacket/conversionutils.h"

#include "CommonStream/packetformat.h"

#include "opengev_global.h"

#include "Application/streamdatareceiver.h"

/**
 * @brief The StreamImageDataTrailerHandler class implements stream image data
 * handler for trailer packet (close PixelData on a stream channel receiver)
 */
class StreamImageDataTrailerHandler : public AbstractStreamDataHandler
{
public:

    /**
     * @brief StreamImageDataTrailerHandler
     * @param target stream channel receiver where PixelMap will be closed
     * @param receivedDatagram
     * @param senderAddress
     * @param senderPort
     */
    StreamImageDataTrailerHandler(GVComponent* target,
                               const QByteArray &receivedDatagram,
                               QHostAddress senderAddress,
                               quint16 senderPort);

    /**
     * @brief execute try to close a new PixelMap on the target stream channel receiver
     * where the received data is stored
     * @return 0 if executed successful
     */
    int execute();
};

#endif // STREAMIMAGEDATATRAILERHANDLER_H

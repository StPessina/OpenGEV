#ifndef STREAMIMAGEDATATRAILER_H
#define STREAMIMAGEDATATRAILER_H

#include "CommonStream/abstractstreamdata.h"
#include "CommonStream/payloadtype.h"

/**
 * @brief The StreamImageDataTrailer class implements data leader packet
 * for image data stream type (CR-290s)
 */
class StreamImageDataTrailer : public AbstractStreamData
{
    friend class StreamImageDataAllIn;
public:
    StreamImageDataTrailer(QHostAddress destAddress, quint16 destPort,
                         quint64 blockId64, quint32 packetId32,
                           quint32 sizey);

    virtual ~StreamImageDataTrailer();

protected:

    virtual quint16 getPacketBodyLength();

    virtual void appendPacketBody(QByteArray &datagram);

private:

    quint32 sizey;
};

#endif // STREAMIMAGEDATATRAILER_H

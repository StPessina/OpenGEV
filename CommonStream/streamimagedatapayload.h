#ifndef STREAMIMAGEDATAPAYLOAD_H
#define STREAMIMAGEDATAPAYLOAD_H

#include "CommonStream/abstractstreamdata.h"

/**
 * @brief The StreamImageDataPayload class implements payload packet for
 * streaming image data (CR-289s)
 */
class StreamImageDataPayload : public AbstractStreamData
{
    friend class StreamImageDataAllIn;
public:

    StreamImageDataPayload(QHostAddress destAddress, quint16 destPort,
                         quint64 blockId64, quint32 packetId32,
                         const char *payloadChar, quint32 payloadCharLength);

    StreamImageDataPayload(QHostAddress destAddress, quint16 destPort,
                                               quint64 blockId64, quint32 packetId32,
                                               QByteArray &payloadChar);

    //virtual void renew(quint32 packetId32, QByteArray data);

    virtual ~StreamImageDataPayload();

protected:

    virtual quint16 getLengthWithoutHeader();

    virtual void appendPacketDatagramWithoutHeader(QByteArray &datagram);

private:

    bool charMode = false;

    const char *payloadChar;

    quint32 payloadCharLength;

    QByteArray payloadArray;
};

#endif // STREAMIMAGEDATAPAYLOAD_H

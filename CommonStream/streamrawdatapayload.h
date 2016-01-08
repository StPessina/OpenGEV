#ifndef STREAMRAWDATAPAYLOAD_H
#define STREAMRAWDATAPAYLOAD_H

#include "CommonStream/abstractstreamdata.h"

class StreamRawDataPayload : public AbstractStreamData
{
public:
    StreamRawDataPayload(QHostAddress destAddress, quint16 destPort,
                         quint64 blockId64, quint32 packetId32,
                         const QByteArray &datagram);

    virtual ~StreamRawDataPayload();

protected:

    virtual quint16 getPacketBodyLength();

    virtual void appendPacketBody(QByteArray &datagram);

private:

    const QByteArray &data;

    quint32 dataByteLength;
};

#endif // STREAMRAWDATAPAYLOAD_H

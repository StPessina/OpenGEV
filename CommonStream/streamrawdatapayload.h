#ifndef STREAMRAWDATAPAYLOAD_H
#define STREAMRAWDATAPAYLOAD_H

#include "CommonStream/abstractstreamdata.h"

class StreamRawDataPayload : public AbstractStreamData
{
public:
    StreamRawDataPayload(QHostAddress destAddress, quint16 destPort,
                         quint64 blockId64, quint32 packetId32,
                         QByteArray datagram);

    virtual ~StreamRawDataPayload();

    virtual int executeAnswer(QByteArray answer);

protected:

    virtual quint16 getLengthWithoutHeader();

    virtual QByteArray getPacketDatagramWithoutHeader();

private:

    QByteArray data;

    quint32 dataByteLength;
};

#endif // STREAMRAWDATAPAYLOAD_H

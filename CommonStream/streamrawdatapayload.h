#ifndef STREAMRAWDATAPAYLOAD_H
#define STREAMRAWDATAPAYLOAD_H

#include "CommonStream/abstractstreamdata.h"

class StreamRawDataPayload : public AbstractStreamData
{
public:
    StreamRawDataPayload(QHostAddress destAddress, quint16 destPort,
                         quint64 blockId64, quint32 packetId32,
                         char* data, quint32 dataByteLength);

    virtual ~StreamRawDataPayload();

    virtual int executeAnswer(QByteArray answer);

protected:

    virtual quint16 getLengthWithoutHeader();

    virtual char* getPacketDatagramWithoutHeader();

private:

    char* data;

    quint32 dataByteLength;
};

#endif // STREAMRAWDATAPAYLOAD_H

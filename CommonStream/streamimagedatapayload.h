#ifndef STREAMIMAGEDATAPAYLOAD_H
#define STREAMIMAGEDATAPAYLOAD_H

#include "CommonStream/abstractstreamdata.h"

/**
 * @brief The StreamImageDataPayload class implements payload packet for
 * streaming image data (CR-289s)
 */
class StreamImageDataPayload : public AbstractStreamData
{
public:
    StreamImageDataPayload(QHostAddress destAddress, quint16 destPort,
                         quint64 blockId64, quint32 packetId32,
                         QByteArray data);

    virtual ~StreamImageDataPayload();

    virtual int executeAnswer(QByteArray answer);

protected:

    virtual quint16 getLengthWithoutHeader();

    virtual QByteArray getPacketDatagramWithoutHeader();

private:

    QByteArray data;

    //quint32 dataByteLength;
};

#endif // STREAMIMAGEDATAPAYLOAD_H

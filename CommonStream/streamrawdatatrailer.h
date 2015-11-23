#ifndef STREAMRAWDATATRAILER_H
#define STREAMRAWDATATRAILER_H

#include "CommonStream/abstractstreamdata.h"
#include "CommonStream/payloadtype.h"

/**
 * @brief The StreamRawDataTrailer class implements data leader packet
 * for raw data stream type (CR-296s)
 */
class StreamRawDataTrailer : public AbstractStreamData
{
public:
    StreamRawDataTrailer(QHostAddress destAddress, quint16 destPort,
                         quint64 blockId64, quint32 packetId32);

    virtual ~StreamRawDataTrailer();

    virtual int executeAnswer(QByteArray answer);

protected:

    virtual quint16 getLengthWithoutHeader();

    virtual QByteArray getPacketDatagramWithoutHeader();
};

#endif // STREAMRAWDATATRAILER_H

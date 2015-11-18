#ifndef STREAMRAWDATALEADER_H
#define STREAMRAWDATALEADER_H

#include "CommonStream/abstractstreamdata.h"
#include "CommonStream/payloadtype.h"


/**
 * @brief The StreamRawDataLeader class implements data leader packet
 * for raw data stream type (CR-294s)
 */
class StreamRawDataLeader : public AbstractStreamData
{
public:
    StreamRawDataLeader(QHostAddress destAddress, quint16 destPort,
                        quint64 blockId64, quint32 packetId32, quint64 payloadSize);

    virtual ~StreamRawDataLeader();

    virtual int executeAnswer(QByteArray answer);

    virtual quint64 getPayloadSize() final;

protected:

    virtual quint16 getLengthWithoutHeader();

    virtual char* getPacketDatagramWithoutHeader();

private:
    quint64 payloadSize;
};

#endif // STREAMRAWDATALEADER_H

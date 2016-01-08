#include "streamrawdatapayload.h"

StreamRawDataPayload::StreamRawDataPayload(QHostAddress destAddress, quint16 destPort,
                                           quint64 blockId64, quint32 packetId32,
                                           const QByteArray &datagram)
    : AbstractStreamData(destAddress, destPort, PacketFormat::DATA_PAYLOAD_GENIRIC_FORMAT,
                         blockId64, packetId32),
    data(datagram)
{
}

StreamRawDataPayload::~StreamRawDataPayload()
{
}

quint16 StreamRawDataPayload::getPacketBodyLength()
{
    return data.size();
}

void StreamRawDataPayload::appendPacketBody(QByteArray &datagram)
{
    datagram.append(data);
}

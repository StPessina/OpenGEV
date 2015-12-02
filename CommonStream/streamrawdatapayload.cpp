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

quint16 StreamRawDataPayload::getLengthWithoutHeader()
{
    return data.size();
}

void StreamRawDataPayload::appendPacketDatagramWithoutHeader(QByteArray &datagram)
{
    datagram.append(data);
}

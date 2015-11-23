#include "streamrawdatapayload.h"

StreamRawDataPayload::StreamRawDataPayload(QHostAddress destAddress, quint16 destPort,
                                           quint64 blockId64, quint32 packetId32,
                                           QByteArray datagram)
    : AbstractStreamData(destAddress, destPort, PacketFormat::DATA_PAYLOAD_GENIRIC_FORMAT,
                         blockId64, packetId32)
{
    this->data = datagram;
}

StreamRawDataPayload::~StreamRawDataPayload()
{
}

int StreamRawDataPayload::executeAnswer(QByteArray answer)
{
    return 0; //No answer required
}

quint16 StreamRawDataPayload::getLengthWithoutHeader()
{
    return data.size();
}

QByteArray StreamRawDataPayload::getPacketDatagramWithoutHeader()
{
    return data;
}

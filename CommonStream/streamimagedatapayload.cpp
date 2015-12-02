#include "streamimagedatapayload.h"

StreamImageDataPayload::StreamImageDataPayload(QHostAddress destAddress, quint16 destPort,
                                           quint64 blockId64, quint32 packetId32,
                                           const char *payload, quint32 dataLength)
    : AbstractStreamData(destAddress, destPort, PacketFormat::DATA_PAYLOAD_GENIRIC_FORMAT,
                         blockId64, packetId32),
      payload(payload), dataLength(dataLength)
{
}

/*
void StreamImageDataPayload::renew(quint32 packetId32, QByteArray data)
{
    this->packetId32=packetId32;
    this->data = data;
    //this->dataByteLength = dataByteLength;
}
*/

StreamImageDataPayload::~StreamImageDataPayload()
{

}

quint16 StreamImageDataPayload::getLengthWithoutHeader()
{
    return dataLength;
    //return payload.size();
}

void StreamImageDataPayload::appendPacketDatagramWithoutHeader(QByteArray &datagram)
{

    datagram.append(payload,getLengthWithoutHeader());
}

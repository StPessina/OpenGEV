#include "streamimagedatapayload.h"

StreamImageDataPayload::StreamImageDataPayload(QHostAddress destAddress, quint16 destPort,
                                           quint64 blockId64, quint32 packetId32,
                                           const char *payload, quint32 dataLength)
    : AbstractStreamData(destAddress, destPort, PacketFormat::DATA_PAYLOAD_GENIRIC_FORMAT,
                         blockId64, packetId32),
      payloadChar(payload),
      payloadCharLength(dataLength)
{
    charMode=true;
}

StreamImageDataPayload::StreamImageDataPayload(QHostAddress destAddress, quint16 destPort,
                                           quint64 blockId64, quint32 packetId32,
                                           QByteArray &payload)
    : AbstractStreamData(destAddress, destPort, PacketFormat::DATA_PAYLOAD_GENIRIC_FORMAT,
                         blockId64, packetId32),
      payloadArray(payload)
{
    charMode=false;
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
    if(charMode)
        return payloadCharLength;
    else
        return payloadArray.size();
}

void StreamImageDataPayload::appendPacketDatagramWithoutHeader(QByteArray &datagram)
{
    if(charMode)
        datagram.append(payloadChar, payloadCharLength);
    else
        datagram.append(payloadArray);
}

#include "streamimagedatapayload.h"

StreamImageDataPayload::StreamImageDataPayload(QHostAddress destAddress, quint16 destPort,
                                           quint64 blockId64, quint32 packetId32,
                                           char* data, quint32 dataByteLength)
    : AbstractStreamData(destAddress, destPort, PacketFormat::DATA_PAYLOAD_GENIRIC_FORMAT,
                         blockId64, packetId32)
{
    this->data = data;
    this->dataByteLength = dataByteLength;
}

StreamImageDataPayload::~StreamImageDataPayload()
{

}

int StreamImageDataPayload::executeAnswer(QByteArray answer)
{
    return 0; //No answer required
}

quint16 StreamImageDataPayload::getLengthWithoutHeader()
{
    return dataByteLength;
}

char *StreamImageDataPayload::getPacketDatagramWithoutHeader()
{
    return data;
}

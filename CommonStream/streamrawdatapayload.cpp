#include "streamrawdatapayload.h"

StreamRawDataPayload::StreamRawDataPayload(QHostAddress destAddress, quint16 destPort,
                                           quint64 blockId64, quint32 packetId32,
                                           char* data, quint32 dataByteLength)
    : AbstractStreamData(destAddress, destPort, PacketFormat::DATA_PAYLOAD_GENIRIC_FORMAT,
                         blockId64, packetId32)
{
    this->data = data;
    this->dataByteLength = dataByteLength;
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
    return dataByteLength;
}

char *StreamRawDataPayload::getPacketDatagramWithoutHeader()
{
    return data;
}

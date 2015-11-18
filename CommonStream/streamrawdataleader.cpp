#include "streamrawdataleader.h"

StreamRawDataLeader::StreamRawDataLeader(GVComponent* target, QHostAddress destAddress, quint16 destPort,
                                         quint64 blockId64, quint32 packetId32, quint64 payloadSize)
    : AbstractStreamData(target, destAddress, destPort, PacketFormat::DATA_LEADER_FORMAT, blockId64, packetId32)
{
    this->payloadSize = payloadSize;
}

int StreamRawDataLeader::executeAnswer(QByteArray answer)
{
    return 0; //No answer required
}


quint64 StreamRawDataLeader::getPayloadSize()
{
    return payloadSize;
}

quint16 StreamRawDataLeader::getLengthWithoutHeader()
{
    return 20;
}

char *StreamRawDataLeader::getPacketDatagramWithoutHeader()
{
    char* datagram = new char[getLengthWithoutHeader()];

    datagram[0]=0; //Reserved
    datagram[1]=0; //Reserved

    ConversionUtils::setShortToCharArray(datagram, PayloadType::RAW_DATA, 2);

    ConversionUtils::setLongToCharArray(datagram, 0, 4); //Should be timestamp

    ConversionUtils::setLongToCharArray(datagram, payloadSize, 12);

    return datagram;
}

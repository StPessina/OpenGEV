#include "streamrawdataleader.h"

StreamRawDataLeader::StreamRawDataLeader(QHostAddress destAddress, quint16 destPort,
                                         quint64 blockId64, quint32 packetId32, quint64 payloadSize)
    : AbstractStreamData(destAddress, destPort, PacketFormat::DATA_LEADER_FORMAT, blockId64, packetId32)
{
    this->payloadSize = payloadSize;
}

StreamRawDataLeader::~StreamRawDataLeader()
{

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

QByteArray StreamRawDataLeader::getPacketDatagramWithoutHeader()
{
    char datagram[getLengthWithoutHeader()];

    datagram[0]=0; //Reserved
    datagram[1]=0; //Reserved

    ConversionUtils::setShortToCharArray(datagram, PayloadType::RAW_DATA, 2);

    ConversionUtils::setLongToCharArray(datagram, 0, 4); //Should be timestamp

    ConversionUtils::setLongToCharArray(datagram, payloadSize, 12);

    QByteArray body (datagram, getLengthWithoutHeader());
    return body;
}

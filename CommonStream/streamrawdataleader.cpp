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

quint64 StreamRawDataLeader::getPayloadSize()
{
    return payloadSize;
}

quint16 StreamRawDataLeader::getPacketBodyLength()
{
    return 20;
}

void StreamRawDataLeader::appendPacketBody(QByteArray &datagram)
{
    ConversionUtils::appendShortToQByteArray(datagram, 0); //Reserved

    ConversionUtils::appendLongToQByteArray(datagram, 0); //Should be timestamp

    ConversionUtils::appendLongToQByteArray(datagram, payloadSize);

}

#include "streamrawdatatrailer.h"

StreamRawDataTrailer::StreamRawDataTrailer(QHostAddress destAddress,
                                           quint16 destPort, quint64 blockId64, quint32 packetId32)
    : AbstractStreamData(destAddress, destPort, PacketFormat::DATA_TRAILER_FORMAT,
                         blockId64, packetId32)
{
}

StreamRawDataTrailer::~StreamRawDataTrailer()
{

}

quint16 StreamRawDataTrailer::getPacketBodyLength()
{
    return 4;
}

void StreamRawDataTrailer::appendPacketBody(QByteArray &datagram)
{    
    ConversionUtils::appendShortToQByteArray(datagram, 0);

    ConversionUtils::appendShortToQByteArray(datagram, PayloadType::RAW_DATA);
}

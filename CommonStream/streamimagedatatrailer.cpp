#include "streamimagedatatrailer.h"

StreamImageDataTrailer::StreamImageDataTrailer(QHostAddress destAddress,
                                           quint16 destPort, quint64 blockId64, quint32 packetId32,
                                               quint32 sizey)
    : AbstractStreamData(destAddress, destPort, PacketFormat::DATA_TRAILER_FORMAT,
                         blockId64, packetId32)
{
    this->sizey = sizey;
}

StreamImageDataTrailer::~StreamImageDataTrailer()
{

}

quint16 StreamImageDataTrailer::getLengthWithoutHeader()
{
    return 8;
}

void StreamImageDataTrailer::appendPacketDatagramWithoutHeader(QByteArray &datagram)
{
    ConversionUtils::appendShortToQByteArray(datagram, 0); //Reserved

    ConversionUtils::appendShortToQByteArray(datagram, PayloadType::IMAGE);

    ConversionUtils::appendIntToQByteArray(datagram, sizey);
}

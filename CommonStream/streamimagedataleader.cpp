#include "streamimagedataleader.h"

StreamImageDataLeader::StreamImageDataLeader(QHostAddress destAddress,
                                             quint16 destPort, quint64 blockId64,
                                             quint32 packetId32, quint32 pixelFormat,
                                             quint32 sizex, quint32 sizey,
                                             quint32 offsetx, quint32 offsety,
                                             quint16 paddingx, quint16 paddingy)
    : AbstractStreamData(destAddress, destPort, PacketFormat::DATA_LEADER_FORMAT, blockId64, packetId32)
{
    this->pixelFormat = pixelFormat;
    this->sizex = sizex;
    this->sizey = sizey;
    this->offsetx = offsetx;
    this->offsety = offsety;
    this->paddingx = paddingx;
    this->paddingy = paddingy;
}

StreamImageDataLeader::~StreamImageDataLeader()
{

}

quint16 StreamImageDataLeader::getLengthWithoutHeader()
{
    return 36;
}

void StreamImageDataLeader::appendPacketDatagramWithoutHeader(QByteArray &datagram)
{    
    ConversionUtils::appendShortToQByteArray(datagram,0);

    ConversionUtils::appendShortToQByteArray(datagram, PayloadType::IMAGE);

    ConversionUtils::appendLongToQByteArray(datagram, 0); //should be timestamp..

    ConversionUtils::appendIntToQByteArray(datagram, pixelFormat);

    ConversionUtils::appendIntToQByteArray(datagram, sizex);
    ConversionUtils::appendIntToQByteArray(datagram, sizey);
    ConversionUtils::appendIntToQByteArray(datagram, offsetx);
    ConversionUtils::appendIntToQByteArray(datagram, offsety);
    ConversionUtils::appendShortToQByteArray(datagram, paddingy);
    ConversionUtils::appendShortToQByteArray(datagram, paddingy);
}

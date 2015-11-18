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

int StreamImageDataLeader::executeAnswer(QByteArray answer)
{
    return 0;
}

quint16 StreamImageDataLeader::getLengthWithoutHeader()
{
    return 36;
}

char *StreamImageDataLeader::getPacketDatagramWithoutHeader()
{
    char* datagram = new char[getLengthWithoutHeader()];

    datagram[0] = 0; //Field id and field count
    datagram[1] = 0; //Reserved

    ConversionUtils::setShortToCharArray(datagram, PayloadType::IMAGE,2);

    ConversionUtils::setLongToCharArray(datagram, 0, 4); //should be timestamp..

    ConversionUtils::setIntToCharArray(datagram, pixelFormat, 12);

    ConversionUtils::setIntToCharArray(datagram, sizex, 16);
    ConversionUtils::setIntToCharArray(datagram, sizey, 20);
    ConversionUtils::setIntToCharArray(datagram, offsetx, 24);
    ConversionUtils::setIntToCharArray(datagram, offsety, 28);
    ConversionUtils::setShortToCharArray(datagram, paddingy, 32);
    ConversionUtils::setShortToCharArray(datagram, paddingy, 34);

}

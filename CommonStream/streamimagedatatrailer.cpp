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

int StreamImageDataTrailer::executeAnswer(QByteArray answer)
{
    return 0; //No answer required
}

quint16 StreamImageDataTrailer::getLengthWithoutHeader()
{
    return 8;
}

QByteArray StreamImageDataTrailer::getPacketDatagramWithoutHeader()
{
    char datagram[getLengthWithoutHeader()];

    datagram[0] = 0; //reserved
    datagram[1] = 0; //reserved

    ConversionUtils::setShortToCharArray(datagram, PayloadType::IMAGE, 2);

    ConversionUtils::setIntToCharArray(datagram, sizey, 4);

    QByteArray body (datagram,getLengthWithoutHeader());
    return body;
}

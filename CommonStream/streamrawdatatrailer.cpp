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

int StreamRawDataTrailer::executeAnswer(QByteArray answer)
{
    return 0; //No answer required
}

quint16 StreamRawDataTrailer::getLengthWithoutHeader()
{
    return 4;
}

char *StreamRawDataTrailer::getPacketDatagramWithoutHeader()
{
    char* datagram = new char[getLengthWithoutHeader()];

    datagram[0] = 0; //reserved
    datagram[1] = 0; //reserved

    ConversionUtils::setShortToCharArray(datagram, PayloadType::RAW_DATA, 2);

    return datagram;
}

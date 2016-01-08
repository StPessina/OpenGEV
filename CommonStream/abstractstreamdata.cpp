#include "abstractstreamdata.h"

AbstractStreamData::AbstractStreamData(QHostAddress destAddress, quint16 destPort,
        PacketFormat packetFormat, quint64 blockId64, quint32 packetId32)
    : AbstractPacket(NULL, destAddress, destPort, 0, false, false)
{
    this->packetFormat = packetFormat;
    this->blockId64 = blockId64;
    this->packetId32 = packetId32;
}

AbstractStreamData::~AbstractStreamData()
{

}

int AbstractStreamData::executeAnswer(const QByteArray &answer)
{
    return 0; //No answer required
}

quint16 AbstractStreamData::getPacketHeaderLength()
{
    return 20;
}

void AbstractStreamData::appendPacketHeader(QByteArray &datagram)
{
    char header[getPacketHeaderLength()];
    ConversionUtils::setShortToCharArray(header, status, 0);

    short flags = getHeaderFlag();
    ConversionUtils::setShortToCharArray(header, flags, 2);

    header[4]=(packetFormat & 0x0F);
    if(extendDI)
        header[4] |= 0x80;

    //Reserved
    header[5]=0;
    header[6]=0;
    header[7]=0;

    ConversionUtils::setLongToCharArray(header, blockId64, 8);

    ConversionUtils::setIntToCharArray(header, packetId32, 16);

    datagram.append(header, getPacketHeaderLength());
}

void AbstractStreamData::setFlagPacketResend()
{
    flagPacketResend = true;
}

short AbstractStreamData::getHeaderFlag()
{
    if(!flagPacketResend)
        return 0;
    else
        return 0x0001;
}

std::string AbstractStreamData::toString()
{
    return "STREAM_DATA /"
            + getDestinationAddress().toString().toStdString() + ":" + std::to_string((int) getDestionationPort()) + "/"
            + std::to_string(isAckRequired()) + "/"
            + std::to_string(isBroadcastMessage()) + "/"
            + std::to_string(packetFormat) + "/"
            + std::to_string(getBlockId64()) + "/"
            + std::to_string(getPacketId32());
}

quint64 AbstractStreamData::getBlockId64()
{
    return blockId64;
}

quint32 AbstractStreamData::getPacketId32()
{
    return packetId32;
}

PacketFormat AbstractStreamData::getPacketFormat()
{
    return packetFormat;
}



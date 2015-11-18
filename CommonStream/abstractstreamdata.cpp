#include "abstractstreamdata.h"

AbstractStreamData::AbstractStreamData(GVComponent *target, QHostAddress destAddress, quint16 destPort,
        PacketFormat packetFormat, quint64 blockId64, quint32 packetId32)
    : AbstractPacket(target, destAddress, destPort, 0, false, false)
{
    this->packetFormat = packetFormat;
    this->blockId64 = blockId64;
    this->packetId32 = packetId32;
}

AbstractStreamData::~AbstractStreamData()
{

}

quint16 AbstractStreamData::getHeaderLength()
{
    return 8;
}

char* AbstractStreamData::getHeader()
{
    char* header = new char[getHeaderLength()];
    header[0]=0x42;
    header[1]=getHeaderFlag();

    header[2]=0; //commandCode >> 8;
    header[3]=0; //commandCode;

    int length = getLengthWithoutHeader();
    header[4]=length >> 8;
    header[5]=length;

    header[6]=getRequestId() >> 8;
    header[7]=getRequestId();
    return header;
}

short AbstractStreamData::getHeaderFlag()
{
    if(isAckRequired())
        return 0x1;
    else
        return 0x0;
}

std::string AbstractStreamData::toString()
{
    return "STREAM_DATA /"
            + getDestinationAddress().toString().toStdString() + ":" + std::to_string((int) getDestionationPort()) + "/"
            + std::to_string(getRequestId()) + "/"
            //+ std::to_string(commandCode) + "/"
            + std::to_string(isAckRequired()) + "/"
            + std::to_string(isBroadcastMessage());
}

quint64 AbstractStreamData::getBlockId64()
{
    return blockId64;
}

quint32 AbstractStreamData::getPacketId32()
{
    return packetId32;
}



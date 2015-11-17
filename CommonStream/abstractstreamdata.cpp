#include "abstractstreamdata.h"

AbstractStreamData::AbstractStreamData(GVComponent *target, QHostAddress destAddress, quint16 destPort,
        quint16 commandCode, quint16 ackCommandCode)
    : AbstractPacket(target, destAddress, destPort, 0, false, false)
{
    this->commandCode=commandCode;
    this->ackCommandCode=ackCommandCode;
}

AbstractStreamData::~AbstractStreamData()
{

}


quint16 AbstractStreamData::getCommandCode()
{
    return commandCode;
}

bool AbstractStreamData::checkAckHeader(QByteArray answer)
{
    if(answer.size()<HEADER_LENGTH)
        return false;
    if(ackCommandCode!=ConversionUtils::getShortFromQByteArray(answer,2))
        return false;
    if(getRequestId()!=ConversionUtils::getShortFromQByteArray(answer,6))
        return false;
    return true;
}

short AbstractStreamData::getStatusCodeFromAnswer(QByteArray answer)
{
    short statusCode = ConversionUtils::getShortFromQByteArray(answer,0);

    return statusCode;
}

short AbstractStreamData::getStatusCode()
{
    return getStatusCodeFromAnswer(answer);
}

quint16 AbstractStreamData::getHeaderLength()
{
    return HEADER_LENGTH;
}

char* AbstractStreamData::getHeader()
{
    char* header = new char[HEADER_LENGTH];
    header[0]=0x42;
    header[1]=getHeaderFlag();

    header[2]=commandCode >> 8;
    header[3]=commandCode;

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
    return getDestinationAddress().toString().toStdString() + ":" + std::to_string((int) getDestionationPort()) + "/"
            + std::to_string(getRequestId()) + "/"
            + std::to_string(commandCode) + "/"
            + std::to_string(isAckRequired()) + "/"
            + std::to_string(isBroadcastMessage());
}



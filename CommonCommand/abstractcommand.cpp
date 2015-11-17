#include "abstractcommand.h"

AbstractCommand::AbstractCommand(GVComponent *target, QHostAddress destAddress, quint16 destPort,
        quint16 commandCode, quint16 ackCommandCode, quint16 reqId, bool requireAck, bool broadcast)
    : AbstractPacket(target, destAddress, destPort, reqId, requireAck, broadcast)
{
    this->commandCode=commandCode;
    this->ackCommandCode=ackCommandCode;
}

AbstractCommand::~AbstractCommand()
{

}


quint16 AbstractCommand::getCommandCode()
{
    return commandCode;
}

bool AbstractCommand::checkAckHeader(QByteArray answer)
{
    if(answer.size()<HEADER_LENGTH)
        return false;
    if(ackCommandCode!=ConversionUtils::getShortFromQByteArray(answer,2))
        return false;
    if(getRequestId()!=ConversionUtils::getShortFromQByteArray(answer,6))
        return false;
    return true;
}

short AbstractCommand::getStatusCodeFromAnswer(QByteArray answer)
{
    short statusCode = ConversionUtils::getShortFromQByteArray(answer,0);

    return statusCode;
}

short AbstractCommand::getStatusCode()
{
    return getStatusCodeFromAnswer(answer);
}

quint16 AbstractCommand::getHeaderLength()
{
    return HEADER_LENGTH;
}

char* AbstractCommand::getHeader()
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

short AbstractCommand::getHeaderFlag()
{
    if(isAckRequired())
        return 0x1;
    else
        return 0x0;
}

std::string AbstractCommand::toString()
{
    return getDestinationAddress().toString().toStdString() + ":" + std::to_string((int) getDestionationPort()) + "/"
            + std::to_string(getRequestId()) + "/"
            + std::to_string(commandCode) + "/"
            + std::to_string(isAckRequired()) + "/"
            + std::to_string(isBroadcastMessage());
}



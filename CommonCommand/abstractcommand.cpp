#include "abstractcommand.h"

AbstractCommand::AbstractCommand(GVComponent * const target, QHostAddress destAddress, quint16 destPort,
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

bool AbstractCommand::checkAckHeader(const QByteArray &answer)
{
    if(answer.size()<getPacketHeaderLength())
        return false;
    if(ackCommandCode!=ConversionUtils::getShortFromQByteArray(answer,2))
        return false;
    if(getRequestId()!=ConversionUtils::getShortFromQByteArray(answer,6))
        return false;
    return true;
}

short AbstractCommand::getStatusCodeFromAnswer(const QByteArray &answer)
{
    short statusCode = ConversionUtils::getShortFromQByteArray(answer,0);

    return statusCode & 0x00FF;
}

short AbstractCommand::getStatusCode()
{
    return getStatusCodeFromAnswer(*answer);
}

quint16 AbstractCommand::getPacketHeaderLength()
{
    return 8;
}

void AbstractCommand::appendPacketHeader(QByteArray &datagram)
{
    datagram.append(0x42);
    datagram.append(getHeaderFlag());

    datagram.append(commandCode >> 8);
    datagram.append(commandCode);

    int length = getPacketBodyLength();
    datagram.append(length >> 8);
    datagram.append(length);

    datagram.append(getRequestId() >> 8);
    datagram.append(getRequestId());
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
    return "COMMAND /"
            + getDestinationAddress().toString().toStdString() + ":" + std::to_string((int) getDestionationPort()) + "/"
            + std::to_string(getRequestId()) + "/"
            + std::to_string(commandCode) + "/"
            + std::to_string(isAckRequired()) + "/"
            + std::to_string(isBroadcastMessage());
}



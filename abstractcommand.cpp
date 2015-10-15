#include "abstractcommand.h"

AbstractCommand::AbstractCommand(int commandCode, int reqId, bool requireAck)
{
    this->commandCode=commandCode;
    this->reqId = reqId;
    this->requireACK = requireAck;
}

QByteArray* AbstractCommand::getCommandDatagram()
{
    int datagramSize = 20 + getLength(); //20 byte for header
    const char* datagramChar = new char[datagramSize];
    QByteArray* datagram = new QByteArray(datagramChar, datagramSize);
}

int AbstractCommand::getCommandCode()
{
    return commandCode;
}

int AbstractCommand::getRequestId()
{
    return reqId;
}

bool AbstractCommand::isAckRequired()
{
    return requireACK;
}

char* AbstractCommand::getHeader()
{

}

char *AbstractCommand::getHeaderFlag()
{

}

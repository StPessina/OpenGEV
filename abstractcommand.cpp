#include "abstractcommand.h"

AbstractCommand::AbstractCommand(QHostAddress destAddress, quint16 destPort,
        int commandCode, int reqId, bool requireAck, bool broadcast)
{
    this->commandCode=commandCode;
    this->reqId = reqId;
    this->requireACK = requireAck;
    this->broadcast = broadcast;
    this->destAddress = destAddress;
    this->destPort = destPort;
}

QByteArray* AbstractCommand::getCommandDatagram()
{
    int datagramSize = 8 + getLengthWithoutHeader(); //8 byte for header
    char* datagramChar = new char[datagramSize];
    char* header = getHeader();
    for (int i = 0; i < 8; ++i)
        datagramChar[i]=header[i];
    if(datagramSize>8) {
        char* body = getCommandDatagramWithoutHeader();
        for (int i = 8; i < datagramSize; ++i)
            datagramChar[i]=body[i-8];
    }
    QByteArray* datagram = new QByteArray(datagramChar, datagramSize);
    return datagram;
}

QHostAddress AbstractCommand::getDestinationAddress()
{
    return destAddress;
}

quint16 AbstractCommand::getDestionationPort()
{
    return destPort;
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

bool AbstractCommand::isBroadcastMessage()
{
    return broadcast;
}

std::string AbstractCommand::toString()
{
    return destAddress.toString().toStdString() + ":" + std::to_string((int) destPort) + "/"
            + std::to_string(reqId) + "/"
            + std::to_string(commandCode) + "/"
            + std::to_string(requireACK) + "/"
            + std::to_string(broadcast);
}

char* AbstractCommand::getHeader()
{
    char* header = new char[8];
    header[0]=0x42;
    header[1]=getHeaderFlag();

    header[2]=commandCode / 256;
    header[3]=commandCode % 256;

    int length = getLengthWithoutHeader();
    header[4]=length / 256;
    header[5]=length % 256;

    header[6]=reqId / 256;
    header[7]=reqId % 256;
    return header;
}

short AbstractCommand::getHeaderFlag()
{
    if(requireACK)
        return 0x80;
    else
        return 0x0;
}

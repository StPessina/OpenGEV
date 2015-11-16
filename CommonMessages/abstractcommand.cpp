#include "abstractcommand.h"

AbstractCommand::AbstractCommand(GVComponent *target, QHostAddress destAddress, quint16 destPort,
        quint16 commandCode, quint16 ackCommandCode, quint16 reqId, bool requireAck, bool broadcast)
{
    this->target = target;
    this->commandCode=commandCode;
    this->ackCommandCode=ackCommandCode;
    this->reqId = reqId;
    this->requireACK = requireAck;
    this->broadcast = broadcast;
    this->destAddress = destAddress;
    this->destPort = destPort;
}

AbstractCommand::~AbstractCommand()
{

}

QByteArray* AbstractCommand::getCommandDatagram()
{
    int datagramSize = HEADER_LENGTH + getLengthWithoutHeader(); //8 byte for header
    char* datagramChar = new char[datagramSize];
    char* header = getHeader();
    for (int i = 0; i < HEADER_LENGTH; ++i)
        datagramChar[i]=header[i];
    delete header;
    if(datagramSize>HEADER_LENGTH) {
        char* body = getCommandDatagramWithoutHeader();
        for (int i = HEADER_LENGTH; i < datagramSize; ++i)
            datagramChar[i]=body[i-HEADER_LENGTH];
        delete body;
    }
    QByteArray* datagram = new QByteArray(datagramChar, datagramSize);
    delete datagramChar;
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

quint16 AbstractCommand::getCommandCode()
{
    return commandCode;
}

void AbstractCommand::setRequestId(quint16 reqId)
{
    this->reqId = reqId;
}

quint16 AbstractCommand::getRequestId()
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

bool AbstractCommand::checkAckHeader(QByteArray answer)
{
    if(answer.at(0)!=0x42)
        return false;
    if(ackCommandCode!=(answer.at(2)*256+answer.at(3)))
        return false;
    if(reqId!=(answer.at(6)*256+answer.at(7)))
        return false;
    return true;
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
    char* header = new char[HEADER_LENGTH];
    header[0]=0x42;
    header[1]=getHeaderFlag();

    header[2]=commandCode >> 8;
    header[3]=commandCode;

    int length = getLengthWithoutHeader();
    header[4]=length >> 8;
    header[5]=length;

    header[6]=reqId >> 8;
    header[7]=reqId;
    return header;
}

short AbstractCommand::getHeaderFlag()
{
    if(requireACK)
        return 0x1;
    else
        return 0x0;
}

bool AbstractCommand::haveAnswer()
{
    return answer!=NULL;
}

QByteArray AbstractCommand::getAnswer()
{
    return answer;
}



#include "abstractcommandhandler.h"

AbstractCommandHandler::AbstractCommandHandler(GVComponent* target, quint16 ackCode, QByteArray datagram,
                                               QHostAddress senderAddress, quint16 senderPort)
    : AbstractPacketHandler(target, datagram, senderAddress, senderPort)
{
    this->ackCode = ackCode;
    this->datagram = datagram;

    this->requestCommandCode = readRequestCommandCode(&datagram);
    this->requestLength = readRequestRequestLength(&datagram);
    this->reqId = readRequestRequestId(&datagram);
}

AbstractCommandHandler::~AbstractCommandHandler()
{

}

bool AbstractCommandHandler::isAckRequired()
{
    return datagram.at(1) & 1;
}


quint16 AbstractCommandHandler::readRequestCommandCode(QByteArray *datagram)
{
    uint valueMSB = datagram->at(2);
    uint valueLSB = datagram->at(3);
    return valueMSB*256+valueLSB;
}

quint16 AbstractCommandHandler::readRequestRequestLength(QByteArray *datagram)
{
    uint valueMSB = datagram->at(4);
    uint valueLSB = datagram->at(5);
    return valueMSB*256+valueLSB;
}

quint16 AbstractCommandHandler::readRequestRequestId(QByteArray *datagram)
{
    uint valueMSB = datagram->at(6);
    uint valueLSB = datagram->at(7);
    return valueMSB*256+valueLSB;
}

quint16 AbstractCommandHandler::getRequestCommandCode()
{
    return requestCommandCode;
}

quint16 AbstractCommandHandler::getRequestLength()
{
    return requestLength;
}

quint16 AbstractCommandHandler::getRequestId()
{
    return reqId;
}

quint16 AbstractCommandHandler::getResultStatus()
{
    return resultStatus;
}

char *AbstractCommandHandler::getAckHeader()
{
    char* header = new char[8];

    header[0]=resultStatus >> 8;
    header[1]=resultStatus;

    header[2]=ackCode >> 8;
    header[3]=ackCode;

    int length = getAckDatagramLengthWithoutHeader();
    header[4]=length >> 8;
    header[5]=length;

    header[6]=reqId >> 8;
    header[7]=reqId;

    return header;
}

bool AbstractCommandHandler::checkHeader()
{
    return true;
}

std::string AbstractCommandHandler::toString()
{
    return sender.toString().toStdString() + ":" + std::to_string((int) port) + "/"
            + std::to_string(resultStatus) + "/"
            + std::to_string(ackCode) + "/"
            + std::to_string(getAckDatagramLengthWithoutHeader()) + "/"
            + std::to_string(reqId);
}

quint16 AbstractCommandHandler::getAckHeaderLength()
{
    return 8;
}

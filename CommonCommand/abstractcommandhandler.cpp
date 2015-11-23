#include "abstractcommandhandler.h"

AbstractCommandHandler::AbstractCommandHandler(GVComponent* target, quint16 ackCode, QByteArray datagram,
                                               QHostAddress senderAddress, quint16 senderPort)
    : AbstractPacketHandler(target, datagram, senderAddress, senderPort)
{
    this->ackCode = ackCode;
    this->datagram = datagram;

    this->requestCommandCode = readRequestCommandCode(&datagram);
    this->requestLength = readRequestLength(&datagram);
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

quint16 AbstractCommandHandler::readRequestLength(QByteArray *datagram)
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

Status AbstractCommandHandler::getResultStatus()
{
    return resultStatus;
}

QByteArray AbstractCommandHandler::getAckHeader()
{
    char headerChar[getAckHeaderLength()];

    headerChar[0]=resultStatus >> 8;
    headerChar[1]=resultStatus;

    headerChar[2]=ackCode >> 8;
    headerChar[3]=ackCode;

    int length = getAckDatagramLengthWithoutHeader();
    headerChar[4]=length >> 8;
    headerChar[5]=length;

    headerChar[6]=reqId >> 8;
    headerChar[7]=reqId;

    QByteArray header (headerChar,getAckHeaderLength());
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

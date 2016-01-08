#include "abstractcommandhandler.h"

AbstractCommandHandler::AbstractCommandHandler(GVComponent* target, quint16 ackCode,
                                               const QByteArray &receivedDatagram,
                                               QHostAddress senderAddress, quint16 senderPort)
    : AbstractPacketHandler(target, receivedDatagram, senderAddress, senderPort)
{
    this->ackCode = ackCode;
    this->requestCommandCode = readRequestCommandCode(receivedDatagram);
    this->requestLength = readRequestLength(receivedDatagram);
    this->reqId = readRequestRequestId(receivedDatagram);
}

AbstractCommandHandler::~AbstractCommandHandler()
{

}

bool AbstractCommandHandler::isAckRequired()
{
    return receivedDatagram.at(1) & 1;
}


quint16 AbstractCommandHandler::readRequestCommandCode(const QByteArray &datagram)
{
    uint valueMSB = datagram.at(2);
    uint valueLSB = datagram.at(3);
    return valueMSB*256+valueLSB;
}

quint16 AbstractCommandHandler::readRequestLength(const QByteArray &datagram)
{
    uint valueMSB = datagram.at(4);
    uint valueLSB = datagram.at(5);
    return valueMSB*256+valueLSB;
}

quint16 AbstractCommandHandler::readRequestRequestId(const QByteArray &datagram)
{
    uint valueMSB = datagram.at(6);
    uint valueLSB = datagram.at(7);
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

void AbstractCommandHandler::appendAckHeader(QByteArray &datagram)
{
    ConversionUtils::appendShortToQByteArray(datagram, resultStatus);
    ConversionUtils::appendShortToQByteArray(datagram, ackCode);
    ConversionUtils::appendShortToQByteArray(datagram, getAckBodyLength());
    ConversionUtils::appendShortToQByteArray(datagram, reqId);
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
            + std::to_string(getAckBodyLength()) + "/"
            + std::to_string(reqId);
}

quint16 AbstractCommandHandler::getAckHeaderLength()
{
    return 8;
}

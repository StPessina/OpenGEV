#include "abstractpackethandler.h"

AbstractPacketHandler::AbstractPacketHandler(GVComponent* target, const QByteArray &receivedDatagram,
                                               QHostAddress senderAddress, quint16 senderPort)
    : receivedDatagram(receivedDatagram)
{
    this->target=target;
    this->sender = senderAddress;
    this->port = senderPort;
}

AbstractPacketHandler::~AbstractPacketHandler()
{

}

GVComponent* AbstractPacketHandler::getTarget()
{
    return target;
}

const QByteArray &AbstractPacketHandler::getReceivedDatagram()
{
    return receivedDatagram;
}

QHostAddress AbstractPacketHandler::getSenderAddress()
{
    return sender;
}

quint16 AbstractPacketHandler::getSenderPort()
{
    return port;
}

bool AbstractPacketHandler::isAckAllowed()
{
    return !ackNotAllowed;
}

const QByteArray &AbstractPacketHandler::getAckDatagram()
{
    ackDatagram.clear();
    ackDatagram.reserve(getAckHeaderLength() + getAckDatagramLengthWithoutHeader());

    appendAckHeader(ackDatagram);
    appendAckDatagramWithoutHeader(ackDatagram);

    return ackDatagram;
}

std::string AbstractPacketHandler::toString()
{
    return sender.toString().toStdString() + ":" + std::to_string((int) port) + "/"
            + std::to_string(getAckDatagramLengthWithoutHeader()) + "/";
}

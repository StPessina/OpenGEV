#include "abstractpackethandler.h"

AbstractPacketHandler::AbstractPacketHandler(GVComponent* target, QByteArray datagram,
                                               QHostAddress senderAddress, quint16 senderPort)
{
    this->target=target;
    this->datagram = datagram;
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

QByteArray AbstractPacketHandler::getDatagram()
{
    return datagram;
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

QByteArray AbstractPacketHandler::getAckDatagram()
{
    int datagramSize = getAckHeaderLength() + getAckDatagramLengthWithoutHeader();
    char datagramChar[datagramSize];
    QByteArray header = getAckHeader();
    for (int i = 0; i < getAckHeaderLength(); ++i)
        datagramChar[i]=header.at(i);

    if(datagramSize>getAckHeaderLength()) {
        QByteArray body = getAckDatagramWithoutHeader();
        for (int i = getAckHeaderLength(); i < datagramSize; ++i)
            datagramChar[i]=body.at(i-getAckHeaderLength());
    }

    QByteArray datagram (datagramChar, datagramSize);
    return datagram;
}

std::string AbstractPacketHandler::toString()
{
    return sender.toString().toStdString() + ":" + std::to_string((int) port) + "/"
            + std::to_string(getAckDatagramLengthWithoutHeader()) + "/";
}

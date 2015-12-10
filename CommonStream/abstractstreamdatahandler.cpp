#include "abstractstreamdatahandler.h"

AbstractStreamDataHandler::AbstractStreamDataHandler(GVComponent* target, quint32 packetFormat,
                                                     const QByteArray &receivedDatagram,
                                               QHostAddress senderAddress, quint16 senderPort)
    : AbstractPacketHandler(target, receivedDatagram, senderAddress, senderPort)
{
    this->requestPacketFormat = packetFormat;
    this->requestBlockId = readRequestBlockId(receivedDatagram);
    this->requestPacketId = readRequestPacketId(receivedDatagram);
}

AbstractStreamDataHandler::~AbstractStreamDataHandler()
{

}

bool AbstractStreamDataHandler::isAckRequired()
{
    return false;
}


quint32 AbstractStreamDataHandler::readRequestPacketFormat(const QByteArray &datagram)
{
    if(datagram.size()<21)
        return -1;
    return (datagram.at(4) & 0x0F);
}

quint32 AbstractStreamDataHandler::readRequestPacketId(const QByteArray &datagram)
{
    if(datagram.size()<21)
        return -1;
    return ConversionUtils::getIntFromQByteArray(datagram, 16);
}

quint64 AbstractStreamDataHandler::readRequestBlockId(const QByteArray &datagram)
{
    if(datagram.size()<21)
        return -1;
    return ConversionUtils::getLongFromQByteArray(datagram, 8);
}

quint32 AbstractStreamDataHandler::getRequestPacketFormat()
{
    return requestPacketFormat;
}

quint32 AbstractStreamDataHandler::getRequestPacketId()
{
    return requestPacketId;
}

quint64 AbstractStreamDataHandler::getRequestBlockId()
{
    return requestBlockId;
}

bool AbstractStreamDataHandler::isPacketResend()
{
    short flag = ConversionUtils::getShortFromQByteArray(receivedDatagram, 0) & 0x00FF;
    return flag == 1;
}

bool AbstractStreamDataHandler::checkHeader()
{
    if(receivedDatagram.length()>20)
        return true;
    return false;
}

std::string AbstractStreamDataHandler::toString()
{
    return sender.toString().toStdString() + ":" + std::to_string((int) port) + "/"
            + std::to_string(requestPacketFormat) + "/"
            + std::to_string(requestBlockId) + "/"
            + std::to_string(getAckDatagramLengthWithoutHeader()) + "/"
            + std::to_string(requestPacketId);
}

quint16 AbstractStreamDataHandler::getAckHeaderLength()
{
    return 0;
}

void AbstractStreamDataHandler::appendAckHeader(QByteArray &datagram)
{
    //No ack required
}

quint16 AbstractStreamDataHandler::getAckDatagramLengthWithoutHeader()
{
    return 0;
}

void AbstractStreamDataHandler::appendAckDatagramWithoutHeader(QByteArray &datagram)
{
    //Nothing to append
}

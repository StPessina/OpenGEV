#include "abstractpacket.h"

AbstractPacket::AbstractPacket(GVComponent *target, QHostAddress destAddress, quint16 destPort,
                               quint16 reqId, bool requireAck, bool broadcast)
{
    this->target = target;
    this->reqId = reqId;
    this->requireACK = requireAck;
    this->broadcast = broadcast;
    this->destAddress = destAddress;
    this->destPort = destPort;
}

AbstractPacket::~AbstractPacket()
{

}

QByteArray AbstractPacket::getPacketDatagram()
{
    int datagramSize = getHeaderLength() + getLengthWithoutHeader(); //8 byte for header
    char datagramChar[datagramSize];
    QByteArray header = getHeader();
    for (int i = 0; i < getHeaderLength(); ++i)
        datagramChar[i]=header.at(i);
    if(datagramSize>getHeaderLength()) {
        QByteArray body = getPacketDatagramWithoutHeader();
        for (int i = getHeaderLength(); i < datagramSize; ++i)
            datagramChar[i]=body.at(i-getHeaderLength());
    }

    QByteArray datagram(datagramChar, datagramSize);
    return datagram;
}

QHostAddress AbstractPacket::getDestinationAddress()
{
    return destAddress;
}

quint16 AbstractPacket::getDestionationPort()
{
    return destPort;
}

void AbstractPacket::setRequestId(quint16 reqId)
{
    this->reqId = reqId;
}

quint16 AbstractPacket::getRequestId()
{
    return reqId;
}

bool AbstractPacket::isAckRequired()
{
    return requireACK;
}

bool AbstractPacket::isBroadcastMessage()
{
    return broadcast;
}

void AbstractPacket::setAnswer(QByteArray answer)
{
    this->answer = answer;
}

bool AbstractPacket::haveAnswer()
{
    return answer!=NULL;
}

QByteArray AbstractPacket::getAnswer()
{
    return answer;
}

std::string AbstractPacket::toString()
{
    return "ABSTRACT_PACKET /"
            + destAddress.toString().toStdString() + ":" + std::to_string((int) destPort) + "/"
            + std::to_string(reqId) + "/"
            + std::to_string(broadcast);
}



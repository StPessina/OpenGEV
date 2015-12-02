#include "abstractpacket.h"

AbstractPacket::AbstractPacket(GVComponent * const target, QHostAddress destAddress, quint16 destPort,
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

const QByteArray &AbstractPacket::getPacketDatagram()
{
    datagram.clear();
    datagram.reserve(getHeaderLength()+getLengthWithoutHeader());

    appendHeader(datagram);
    appendPacketDatagramWithoutHeader(datagram);
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

void AbstractPacket::setAnswer(const QByteArray &answer)
{
    this->answer = &answer;
}

bool AbstractPacket::haveAnswer()
{
    return answer!=NULL;
}

const QByteArray *AbstractPacket::getAnswer()
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



#include "controlchannel.h"

ControlChannel::ControlChannel(QHostAddress sourceAddr,
                               quint16 sourcePort,
                               AbstractMessageFactory* messageHandlerFactory)
{
    this->sourceAddr = sourceAddr;
    this->sourcePort = sourcePort;
    this->messageHandlerFactory = messageHandlerFactory;
}

void ControlChannel::initSocket()
{
    socket = new QUdpSocket(this);
    socket->bind(sourceAddr, sourcePort);

    connect(socket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));
}

void ControlChannel::setMonitorAccess()
{
    ctrlChannelPrivilege = MONITOR;
}

void ControlChannel::setCtrlAccess(QHostAddress applicationAddr, quint16 applicationPort)
{
    ctrlChannelPrivilege = CTRL_ACCESS;
    this->applicationAddr = applicationAddr;
    this->applicationPort = applicationPort;
}

void ControlChannel::setCtrlAccessSwitchOver(QHostAddress applicationAddr, quint16 applicationPort)
{
    ctrlChannelPrivilege = CTRL_ACCESS_SWITCH_OVER;
    this->applicationAddr = applicationAddr;
    this->applicationPort = applicationPort;
}

void ControlChannel::setExclusiveAccess(QHostAddress applicationAddr, quint16 applicationPort)
{
    ctrlChannelPrivilege = EXCLUSIVE;
    this->applicationAddr = applicationAddr;
    this->applicationPort = applicationPort;
}

Privilege ControlChannel::checkChannelPrivilege(QHostAddress senderAddr, quint16 senderPort)
{
    switch (ctrlChannelPrivilege) {
        case MONITOR:
            return FULL;
        case CTRL_ACCESS:
            if(senderAddr==applicationAddr && senderPort == applicationPort)
                return FULL;
            else
                return READ;
        case CTRL_ACCESS_SWITCH_OVER:
            if(senderAddr==applicationAddr && senderPort == applicationPort)
                return FULL;
            else
                return READ_SWITCH_OVER;
        case EXCLUSIVE:
            if(senderAddr==applicationAddr && senderPort == applicationPort)
                return FULL;
            else
                return DENIED;
        default:
            break;
    }
    return DENIED;
}

void ControlChannel::readPendingDatagrams()
{
    while (socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(socket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        socket->readDatagram(datagram.data(), datagram.size(),
                                &sender, &senderPort);

        processTheDatagram(datagram, sender, senderPort);
    }
}

QHostAddress ControlChannel::getSourceAddress()
{
    return sourceAddr;
}

quint16 ControlChannel::getSourcePort()
{
    return sourcePort;
}

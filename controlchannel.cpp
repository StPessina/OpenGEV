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

bool ControlChannel::checkChannelPrivilege(QHostAddress senderAddr, quint16 senderPort)
{
    switch (ctrlChannelPrivilege) {
        case MONITOR:
        case CTRL_ACCESS:
        case CTRL_ACCESS_SWITCH_OVER:
            return true;
        case EXCLUSIVE:
            if(senderAddr==applicationAddr && senderPort == applicationPort)
                return true;
            break;
        default:
            break;
    }
    return false;
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

void ControlChannel::processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort)
{
    if(checkChannelPrivilege(sender, senderPort)) {
        if(datagram.at(0)==0x42) {

        }
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

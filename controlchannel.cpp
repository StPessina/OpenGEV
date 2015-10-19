#include "controlchannel.h"

ControlChannel::ControlChannel(QHostAddress sourceAddr,
                               quint16 sourcePort,
                               AbstractMessageHandlerFactory *messageHandlerFactory)
{
    this->sourceAddr = sourceAddr;
    this->sourcePort = sourcePort;
    this->messageHandlerFactory = messageHandlerFactory;

    this->timeoutTimer=new QTimer(this);

    logger.infoStream()<<getLogMessageHeader()<<"New";
}

void ControlChannel::initSocket()
{
    socket = new QUdpSocket(this);
    socket->bind(sourceAddr, sourcePort);

    connect(socket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));

    logger.infoStream()<<getLogMessageHeader()<<"Init socket";
}

void ControlChannel::setMonitorAccess()
{
    ctrlChannelPrivilege = MONITOR;
    logger.infoStream()<<getLogMessageHeader()<<"Set monitor level access";
}

void ControlChannel::setCtrlAccess(QHostAddress applicationAddr, quint16 applicationPort)
{
    ctrlChannelPrivilege = CTRL_ACCESS;
    this->applicationAddr = applicationAddr;
    this->applicationPort = applicationPort;
    logger.infoStream()<<getLogMessageHeader()<<"Set control access level; "
                        <<"Control host at "
                        <<applicationAddr.toString().toStdString()
                        <<":"<<(int) applicationPort;

}

void ControlChannel::setCtrlAccessSwitchOver(QHostAddress applicationAddr, quint16 applicationPort)
{
    ctrlChannelPrivilege = CTRL_ACCESS_SWITCH_OVER;
    this->applicationAddr = applicationAddr;
    this->applicationPort = applicationPort;
    logger.infoStream()<<getLogMessageHeader()<<"Set control access switch over level; "
                        <<"Control host at "
                        <<applicationAddr.toString().toStdString()
                        <<":"<<(int) applicationPort;
}

void ControlChannel::setExclusiveAccess(QHostAddress applicationAddr, quint16 applicationPort)
{
    ctrlChannelPrivilege = EXCLUSIVE;
    this->applicationAddr = applicationAddr;
    this->applicationPort = applicationPort;
    logger.infoStream()<<getLogMessageHeader()<<"Set exclusive access level; "
                        <<"Control host at "
                        <<applicationAddr.toString().toStdString()
                        <<":"<<(int) applicationPort;
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

std::string ControlChannel::getLogMessageHeader()
{
    return "Control channel ("
            + sourceAddr.toString().toStdString()
            + ":" + std::to_string((int) sourcePort) + ") - ";
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

        logger.debugStream()<<getLogMessageHeader()<<"New datagram received from "
                            <<sender.toString().toStdString()
                            <<":"<<(int) senderPort<<"; "
                            <<"datagram: "<<datagram.toHex().data()<<" "
                            <<"size: "<<datagram.size()<<" Byte";

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

bool ControlChannel::isSocketOpen()
{
    return true;
}



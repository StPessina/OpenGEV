#include "udpchannel.h"

UDPChannel::UDPChannel(QHostAddress sourceAddr,
                               quint16 sourcePort)
{
    this->sourceAddr = sourceAddr;
    this->sourcePort = sourcePort;

    this->timeoutTimer=new QTimer(this);

    logger.infoStream()<<getLogMessageHeader()<<"New";
}

UDPChannel::~UDPChannel()
{
    delete socket;
    delete timeoutTimer;
}

void UDPChannel::initSocket()
{
    socket = new QUdpSocket(this);
    socket->bind(sourceAddr, sourcePort);

    connect(socket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));

    logger.infoStream()<<getLogMessageHeader()<<"Init socket";
}


std::string UDPChannel::getLogMessageHeader()
{
    return "Control channel ("
            + sourceAddr.toString().toStdString()
            + ":" + std::to_string((int) sourcePort) + ") - ";
}

void UDPChannel::readPendingDatagrams()
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

QHostAddress UDPChannel::getSourceAddress()
{
    return sourceAddr;
}

quint16 UDPChannel::getSourcePort()
{
    return sourcePort;
}

bool UDPChannel::isSocketOpen()
{
    return true;
}



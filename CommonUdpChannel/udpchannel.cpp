#include "udpchannel.h"

UDPChannel::UDPChannel(QHostAddress sourceAddr,
                       quint16 sourcePort)
{
    this->sourceAddr = sourceAddr;
    this->sourcePort = sourcePort;

    this->timeoutTimer=new QTimer(this);

#ifdef ENABLE_LOG4CPP
    logger.infoStream()<<getLogMessageHeader()<<"New";
#endif
}

UDPChannel::~UDPChannel()
{
    delete socket;
    delete timeoutTimer;
}

bool UDPChannel::initSocket()
{
    socket = new QUdpSocket(this);

    if(socket->bind(sourceAddr, sourcePort)) {

        connect(socket, SIGNAL(readyRead()),
                this, SLOT(readPendingDatagrams()));
#ifdef ENABLE_LOG4CPP
        logger.infoStream()<<getLogMessageHeader()<<"Socket bind successful";
#endif

        return true;
    } else {
#ifdef ENABLE_LOG4CPP
        logger.errorStream()<<getLogMessageHeader()<<"Can't init socket";
#endif
        return false;
    }
}

#ifdef ENABLE_LOG4CPP
std::string UDPChannel::getLogMessageHeader()
{
    return "Control channel ("
            + sourceAddr.toString().toStdString()
            + ":" + std::to_string((int) sourcePort) + ") - ";
}
#endif

void UDPChannel::readPendingDatagrams()
{
    while (socket->hasPendingDatagrams()) {
        datagram.resize(socket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        socket->readDatagram(datagram.data(), datagram.size(),
                             &sender, &senderPort);
#ifdef ENABLE_LOG4CPP
        logger.debugStream()<<getLogMessageHeader()<<"New datagram received from "
                           <<sender.toString().toStdString()
                          <<":"<<(int) senderPort<<"; "
                         //<<"datagram: "<<datagram.toHex().data()<<" "
                        <<"size: "<<datagram.size()<<" Byte";
#endif

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



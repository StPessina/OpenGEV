#include "qtudpchannel.h"

QtUDPChannel::QtUDPChannel(QHostAddress sourceAddr,
                       quint16 sourcePort)
    : UDPChannel(sourceAddr, sourcePort)
{
}

QtUDPChannel::QtUDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
                           AbstractPacketHandlerFactory *packetHandlerFactory)
    : UDPChannel(sourceAddr, sourcePort, packetHandlerFactory)
{

}

QtUDPChannel::~QtUDPChannel()
{
    delete socket;
    delete timeoutTimer;
}

bool QtUDPChannel::initSocket()
{
    socket = new QUdpSocket(this);

    if(socket->bind(sourceAddr, sourcePort)) {

        connect(socket, SIGNAL(readyRead()),
                this, SLOT(readPendingDatagrams()));
#ifdef USE_LOG4CPP
        logger.infoStream()<<getLogMessageHeader()<<"Socket bind successful";
#endif

        return true;
    } else {
#ifdef USE_LOG4CPP
        logger.errorStream()<<getLogMessageHeader()<<"Can't init socket";
#endif
        return false;
    }
}

void QtUDPChannel::readPendingDatagrams()
{
    while (socket->hasPendingDatagrams()) {
        datagram.resize(socket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        socket->readDatagram(datagram.data(), datagram.size(),
                             &sender, &senderPort);
#ifdef USE_LOG4CPP
        logger.debugStream()<<getLogMessageHeader()<<"New datagram received from "
                           <<sender.toString().toStdString()
                          <<":"<<(int) senderPort<<"; "
                         //<<"datagram: "<<datagram.toHex().data()<<" "
                        <<"size: "<<datagram.size()<<" Byte";
#endif

        processTheDatagram(datagram, sender, senderPort);
    }
}

int QtUDPChannel::writeDatagram(const QByteArray &datagram, QHostAddress destAddr, quint16 destPort)
{
    return socket->writeDatagram(datagram,destAddr,destPort);
}

bool QtUDPChannel::hasPendingDatagrams()
{
    return socket->hasPendingDatagrams();
}

bool QtUDPChannel::isSocketOpen()
{
    return true;
}

void QtUDPChannel::run()
{
    exec(); //For Qt event support
}



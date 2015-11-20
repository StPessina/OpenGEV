#include "udpchannelreceiver.h"

UdpChannelReceiver::UdpChannelReceiver(QHostAddress sourceAddr,
                                         quint16 sourcePort,
                                         AbstractPacketHandlerFactory *messageHandlerFactory)
    : UDPChannel(sourceAddr,sourcePort)
{
    this->packetHandlerFactory = messageHandlerFactory;
}

UdpChannelReceiver::~UdpChannelReceiver()
{

}

void UdpChannelReceiver::processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort)
{
    quint16 handlerIdentifier = packetHandlerFactory->getPacketHandlerIdentifier(datagram);

    if(!packetHandlerFactory->isValidCode(handlerIdentifier)) {
        logger.warnStream()<<getLogMessageHeader()
                           <<"Unknow handler identifier "
                         <<"Datagram: "<<datagram.toHex().data()<<" "
                        <<"Datagram size: "<<datagram.size();
        return;
    }

    AbstractPacketHandler* packetHandler = packetHandlerFactory->createPacketHandler(handlerIdentifier, datagram,
                                                                              sender, senderPort);
    packetHandler->execute();

    if(packetHandler->isAckRequired()) { //if ack is required
        if(packetHandler->isAckAllowed()) {
            QByteArray* ackDatagram = packetHandler->getAckDatagram();
            socket->writeDatagram(*ackDatagram, sender, senderPort);
            logger.debugStream()<<getLogMessageHeader()
                               <<"Ack sent "
                              <<"("<<packetHandler->toString()<<") "
                             <<"Ack datagram: "<<ackDatagram->toHex().data()<<" "
                            <<"Ack datagram size: "<<ackDatagram->size();
        } else
            logger.debugStream()<<getLogMessageHeader()
                               <<"Ack not allowed from msg handler "
                              <<"("<<packetHandler->toString()<<") "
                             <<"Datagram: "<<datagram.toHex().data()<<" "
                            <<"Datagram size: "<<datagram.size();
    } else {
        logger.debugStream()<<getLogMessageHeader()
                           <<"Ack not required "
                          <<"("<<packetHandler->toString()<<") "
                         <<"Datagram: "<<datagram.toHex().data()<<" "
                        <<"Datagram size: "<<datagram.size();
    }

    delete packetHandler;
}

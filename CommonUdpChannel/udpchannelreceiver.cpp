#include "udpchannelreceiver.h"

UdpChannelReceiver::UdpChannelReceiver(QHostAddress sourceAddr,
                                         quint16 sourcePort,
                                         AbstractMessageHandlerFactory *messageHandlerFactory)
    : UDPChannel(sourceAddr,sourcePort)
{
    this->messageHandlerFactory = messageHandlerFactory;
}

UdpChannelReceiver::~UdpChannelReceiver()
{

}

void UdpChannelReceiver::processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort)
{
    quint16 messageCode = ConversionUtils::getShortFromQByteArray(datagram,2);

    AbstractMessageHandler* msg = messageHandlerFactory->createMessageHandler(messageCode, datagram,
                                                                              sender, senderPort);
    msg->execute();

    bool ackRequired = datagram.at(1) & 1;
    if(ackRequired) { //if ack is required
        if(msg->isAckAllowed()) {
            QByteArray* ackDatagram = msg->getAckDatagram();
            socket->writeDatagram(*ackDatagram, sender, senderPort);
            logger.debugStream()<<getLogMessageHeader()
                               <<"Ack sent "
                              <<"("<<msg->toString()<<") "
                             <<"Ack datagram: "<<ackDatagram->toHex().data()<<" "
                            <<"Ack datagram size: "<<ackDatagram->size();
        } else
            logger.debugStream()<<getLogMessageHeader()
                               <<"Ack not allowed from msg handler "
                              <<"("<<msg->toString()<<") "
                             <<"Datagram: "<<datagram.toHex().data()<<" "
                            <<"Datagram size: "<<datagram.size();
    } else {
        logger.debugStream()<<getLogMessageHeader()
                           <<"Ack not required "
                          <<"("<<msg->toString()<<") "
                         <<"Datagram: "<<datagram.toHex().data()<<" "
                        <<"Datagram size: "<<datagram.size();
    }

    delete msg;
}

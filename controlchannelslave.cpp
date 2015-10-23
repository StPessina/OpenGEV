#include "controlchannelslave.h"

ControlChannelSlave::ControlChannelSlave(QHostAddress sourceAddr,
                                           quint16 sourcePort,
                                           AbstractMessageHandlerFactory *messageHandlerFactory)
    : ControlChannel(sourceAddr,sourcePort,messageHandlerFactory)
{

}

ControlChannelSlave::~ControlChannelSlave()
{

}

void ControlChannelSlave::processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort)
{
    Privilege privilege = checkChannelPrivilege(sender, senderPort);
    int messageCodeMSB = datagram.at(2);
    int messageCodeLSB = datagram.at(3);
    int messageCode = messageCodeMSB*256+messageCodeLSB;

    AbstractMessageHandler* msg = messageHandlerFactory->createMessageHandler(messageCode, datagram,
                                                                                          sender, senderPort);
    msg->execute(privilege);

    bool ackRequired = datagram.at(1) & 1;
    if(ackRequired) { //if ack is required
        QByteArray* ackDatagram = msg->getAckDatagram();
        socket->writeDatagram(*ackDatagram, sender, senderPort);
        logger.debugStream()<<getLogMessageHeader()
                            <<"Ack sent "
                            <<"("<<msg->toString()<<") "
                            <<"Ack datagram: "<<ackDatagram->toHex().data()<<" "
                            <<"Ack datagram size: "<<ackDatagram->size();
    } else {
        logger.debugStream()<<getLogMessageHeader()
                            <<"Ack not required "
                            <<"("<<msg->toString()<<") "
                            <<"Datagram: "<<datagram.toHex().data()<<" "
                            <<"Datagram size: "<<datagram.size();
    }
}

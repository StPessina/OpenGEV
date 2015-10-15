#include "devicecontrolchannel.h"

DeviceControlChannel::DeviceControlChannel(QHostAddress sourceAddr,
                                           quint16 sourcePort,
                                           DeviceMessageFactory *messageHandlerFactory)
    : ControlChannel(sourceAddr,sourcePort,messageHandlerFactory)
{

}

void ControlChannel::processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort)
{
    Privilege privilege = checkChannelPrivilege(sender, senderPort);
    if(privilege!=DENIED) { //Check access to the channel is allowed
        if(datagram.at(0)==0x42) { //Check message header as GV message
            int messageCodeMSB = datagram.at(2);
            int messageCodeLSB = datagram.at(3);
            int messageCode = messageCodeMSB*256+messageCodeLSB;
            if(messageHandlerFactory->isValidCode(messageCode)) { //If the message code exist
                AbstractMessage msg = messageHandlerFactory->createMessageHandler(messageCode);
                if(msg.isAllowed(privilege)) { //If the command is allowed
                    //TODO: PENDING ACK IF CMD EXCUTION IS TOO LONG
                    msg.execute(datagram, sender, senderPort);
                    bool ackRequired = datagram.at(1) & 128;
                    if(ackRequired) { //if ack is required
                        QByteArray ackDatagram = msg.getAck();
                        socket->writeDatagram(ackDatagram, sender, senderPort);
                    }
                } else {
                    //ACK ACCESS DENIED
                }
            }
        }
    }
}

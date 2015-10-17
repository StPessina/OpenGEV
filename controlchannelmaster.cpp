#include "controlchannelmaster.h"

ControlChannelMaster::ControlChannelMaster(QHostAddress sourceAddr,
                                                     quint16 sourcePort,
                                                     AbstractMessageHandlerFactory *messageHandlerFactory)
    : ControlChannel(sourceAddr, sourcePort, messageHandlerFactory)
{
}

void ControlChannelMaster::processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort)
{
    if(waitForAck) {
        logger.debugStream()<<getLogMessageHeader()
                            <<"Ack received "
                            <<"Datagram: "<<datagram.toHex().data()<<" "
                            <<"Datagram size: "<<datagram.size();
        waitForAck = false;
    }
}

void ControlChannelMaster::sendCommand(AbstractCommand *cmd)
{
    if(isSocketOpen()) {
        QByteArray* datagram = cmd->getCommandDatagram();
        logger.debugStream()<<getLogMessageHeader()
                            <<"Require write command "
                            <<"("<<cmd->toString()<<") "
                            <<"Datagram: "<<datagram->toHex().data()<<" "
                            <<"Datagram size: "<<datagram->size();


        int result = socket->writeDatagram(*datagram,
                              cmd->getDestinationAddress(),
                              cmd->getDestionationPort());

        if(result==-1)
            logger.debugStream()<<getLogMessageHeader()
                                <<"Command NOT writed cause error: "<<result<<" "
                                <<"("<<cmd->toString()<<")";
        if(result==0)
            logger.debugStream()<<getLogMessageHeader()
                                <<"Command writed but with 0 byte: "
                                <<"datagram size: "<<datagram->size()<<" "
                                <<"byte sent: "<<result<<" "
                                <<"("<<cmd->toString()<<")";
        if(result>0) {
            if(result==datagram->size()) {
                logger.debugStream()<<getLogMessageHeader()
                                    <<"Command writed "
                                    <<"("<<cmd->toString()<<")";
            } else {
                    logger.debugStream()<<getLogMessageHeader()
                                    <<"Command writed but datagram size mismatch: "
                                    <<"datagram size: "<<datagram->size()<<" "
                                    <<"byte sent: "<<result<<" "
                                    <<"("<<cmd->toString()<<")";
            }
        }
    } else {
        logger.warnStream()<<getLogMessageHeader()
                            <<"Require write cmd "
                            <<"("<<cmd->toString()<<") "
                            <<"but socket is closed";
    }

    if(cmd->isAckRequired()) {
        waitForAck=true;
    }
}


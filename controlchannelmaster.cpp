#include "controlchannelmaster.h"

ControlChannelMaster::ControlChannelMaster(QHostAddress sourceAddr,
                                                     quint16 sourcePort,
                                                     AbstractMessageHandlerFactory *messageHandlerFactory)
    : ControlChannel(sourceAddr, sourcePort, messageHandlerFactory)
{
    connect(timeoutTimer, SIGNAL(timeout()),this,SLOT(timeoutAck()));
}

void ControlChannelMaster::processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort)
{
    if(waitForAck) {
        logger.debugStream()<<getLogMessageHeader()
                            <<"Ack received "
                            <<"Datagram: "<<datagram.toHex().data()<<" "
                            <<"Datagram size: "<<datagram.size();
        int messageAckMSB = datagram.at(2);
        int messageAckLSB = datagram.at(3);
        int messageAck = messageAckMSB*256+messageAckLSB;

        msg = messageHandlerFactory->createMessageHandler(messageAck, datagram,
                                                            sender, senderPort);

        //msg->execute(FULL);

        logger.debugStream()<<getLogMessageHeader()
                            <<"Ack message processed "
                            <<"("<<msg->toString()<<") "
                            <<"Datagram: "<<datagram.toHex().data();
        waitForAck = false;
        emit stopWaitingAck();
    }
}

int ControlChannelMaster::sendCommand(AbstractCommand *cmd)
{
    int result = -3;
    if(isSocketOpen()) {
        QByteArray* datagram = cmd->getCommandDatagram();

        retryCounter = 1;
        while(retryCounter<=RETRY_SEND && result!=datagram->size()) {
            logger.debugStream()<<getLogMessageHeader()
                                <<"Require write command "
                                <<"("<<cmd->toString()<<") "
                                <<"Datagram: "<<datagram->toHex().data()<<" "
                                <<"Datagram size: "<<datagram->size()<<" "
                               <<"(Retry counter: "<<retryCounter<<")";

            result = socket->writeDatagram(*datagram,
                              cmd->getDestinationAddress(),
                              cmd->getDestionationPort());

            if(result==-1)
                logger.debugStream()<<getLogMessageHeader()
                                <<"Command NOT writed cause error: "<<result<<" "
                                <<"("<<cmd->toString()<<"); "
                                <<"(Retry counter: "<<retryCounter<<")";
            if(result==0)
                logger.debugStream()<<getLogMessageHeader()
                                <<"Command writed but with 0 byte: "
                                <<"datagram size: "<<datagram->size()<<" "
                                <<"byte sent: "<<result<<" "
                                <<"("<<cmd->toString()<<") "
                                <<"(Retry counter: "<<retryCounter<<")";
            if(result>0) {
                if(result!=datagram->size()) {
                        logger.debugStream()<<getLogMessageHeader()
                                    <<"Command writed but datagram size mismatch: "
                                    <<"datagram size: "<<datagram->size()<<" "
                                    <<"byte sent: "<<result<<" "
                                    <<"("<<cmd->toString()<<") "
                                    <<"(Retry counter: "<<retryCounter<<")";
                } else {
                    if(cmd->isAckRequired()) {
                        logger.debugStream()<<getLogMessageHeader()
                                            <<"Command writed "
                                            <<"("<<cmd->toString()<<") "
                                            <<"and start waiting for ack"
                                            <<"(Retry counter: "<<retryCounter<<") "
                                            <<"(Timeout: "<<TIMEOUT_MS<<" ms)";
                        waitForAck=true;
                        timeoutExpired=false;

                        timeoutTimer->start(TIMEOUT_MS);

                        QEventLoop loop;
                        connect(this, SIGNAL(stopWaitingAck()), &loop, SLOT(quit()));
                        loop.exec();

                        if(timeoutExpired) {
                            result = -2;
                            logger.debugStream()<<getLogMessageHeader()
                                                <<"Command write "
                                                <<"("<<cmd->toString()<<") "
                                                <<"(Retry counter: "<<retryCounter<<") "
                                                <<"but ack timeout is expired";
                        }

                    } else {
                        logger.debugStream()<<getLogMessageHeader()
                                            <<"Command writed "
                                            <<"("<<cmd->toString()<<") "
                                            <<"(Retry counter: "<<retryCounter<<") "
                                            <<"and no ack is required";
                    }
                }
            }

            retryCounter++;
        }

        if(retryCounter>RETRY_SEND) {
            logger.warnStream()<<getLogMessageHeader()
                                <<"Require write cmd "
                                <<"("<<cmd->toString()<<") "
                                <<"but retry counter is expired, "
                                <<"Retry counter: "<<retryCounter;
        }

    } else {
        logger.warnStream()<<getLogMessageHeader()
                            <<"Require write cmd "
                            <<"("<<cmd->toString()<<") "
                            <<"but socket is closed";
    }

    return result;
}

void ControlChannelMaster::timeoutAck()
{
    timeoutExpired=true;
    emit stopWaitingAck();
}


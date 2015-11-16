#include "controlchannelmaster.h"

ControlChannelMaster::ControlChannelMaster(QHostAddress sourceAddr,
                                                     quint16 sourcePort)
    : UDPChannel(sourceAddr, sourcePort)
{
    connect(timeoutTimer, SIGNAL(timeout()),this,SLOT(timeoutAck()));
}

ControlChannelMaster::~ControlChannelMaster()
{

}

void ControlChannelMaster::processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort)
{
    if(waitForAck) {
        logger.debugStream()<<getLogMessageHeader()
                            <<"Ack received from"
                            <<sender.toString().toStdString()<<":"<<std::to_string(senderPort)<<"; "
                            <<"Datagram: "<<datagram.toHex().data()<<" "
                            <<"Datagram size: "<<datagram.size();
        quint16 ackIdMSB = datagram.at(6);
        quint16 ackIdLSB = datagram.at(7);
        quint16 ackId = ackIdMSB*256+ackIdLSB;

        if(commandCache[ackId]!=NULL) {
            AbstractCommand* reqCmd = commandCache[ackId];
            reqCmd->executeAnswer(datagram);
            logger.debugStream()<<getLogMessageHeader()
                            <<"Ack message processed "
                            <<"("<<reqCmd->toString()<<") "
                            <<"Datagram: "<<datagram.toHex().data();
        } else {
            logger.warnStream()<<getLogMessageHeader()
                            <<"Ack message not processed, not cmd found "
                            <<"Datagram: "<<datagram.toHex().data();
        }
        if(!socket->hasPendingDatagrams()) {
            waitForAck = false;
            emit stopWaitingAck();
        } else timeoutTimer->start(TIMEOUT_MS);
    }
}

int ControlChannelMaster::sendCommand(AbstractCommand *cmd)
{
    int result = -3;
    if(isSocketOpen()) {
        lastReqId++;
        cmd->setRequestId(lastReqId);
        commandCache[lastReqId]=cmd;
        QByteArray* datagram = cmd->getCommandDatagram();

        retryCounter = 1;
        while(retryCounter<=RETRY_SEND && result!=datagram->size()) {
            logger.debugStream()<<getLogMessageHeader()
                                <<"Require write command "
                                <<"("<<cmd->toString()<<") "
                                <<"Datagram: "<<datagram->toHex().data()<<" "
                                <<"Datagram size: "<<datagram->size()<<" "
                               <<"(Retry counter: "<<retryCounter<<")";

            waitForAck=true;
            timeoutExpired=false;

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

                        timeoutTimer->start(TIMEOUT_MS);

                        QEventLoop loop;
                        connect(this, SIGNAL(stopWaitingAck()), &loop, SLOT(quit()));
                        loop.exec();

                        commandCache.erase(cmd->getRequestId());

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


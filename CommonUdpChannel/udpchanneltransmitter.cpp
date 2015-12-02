#include "udpchanneltransmitter.h"

UDPChannelTransmitter::UDPChannelTransmitter(QHostAddress sourceAddr,
                                             quint16 sourcePort)
    : UDPChannel(sourceAddr, sourcePort)
{
    connect(timeoutTimer, SIGNAL(timeout()),this,SLOT(timeoutAck()));
}

UDPChannelTransmitter::~UDPChannelTransmitter()
{

}

void UDPChannelTransmitter::processTheDatagram(QByteArray &datagram, QHostAddress sender, quint16 senderPort)
{
    if(waitForAck) {
#ifdef ENABLE_LOG4CPP
        logger.debugStream()<<getLogMessageHeader()
                           <<"Ack received from"
                          <<sender.toString().toStdString()<<":"<<std::to_string(senderPort)<<"; "
                         <<"Datagram: "<<datagram.toHex().data()<<" "
                        <<"Datagram size: "<<datagram.size();
#endif
        quint16 ackIdMSB = datagram.at(6);
        quint16 ackIdLSB = datagram.at(7);
        quint16 ackId = ackIdMSB*256+ackIdLSB;

        if(packetCache[ackId]!=NULL) {
            AbstractPacket *reqPacket = packetCache[ackId];
            reqPacket->executeAnswer(datagram);
#ifdef ENABLE_LOG4CPP
            logger.debugStream()<<getLogMessageHeader()
                               <<"Ack message processed "
                              <<"("<<reqPacket->toString()<<") "
                             <<"Datagram: "<<datagram.toHex().data();
#endif
        }
#ifdef ENABLE_LOG4CPP
        else {
            logger.warnStream()<<getLogMessageHeader()
                              <<"Ack message not processed, not cmd found "
                             <<"Datagram: "<<datagram.toHex().data();
        }
#endif
        if(!socket->hasPendingDatagrams()) {
            waitForAck = false;
            emit stopWaitingAck();
        }
    }
}

int UDPChannelTransmitter::sendCommand(AbstractPacket &packet)
{
    int result = -3;
    if(isSocketOpen()) {
        if(lastReqId>65000)
            lastReqId=0;
        lastReqId++;
        packet.setRequestId(lastReqId);
        packetCache[lastReqId]=&packet;
        QByteArray datagram = packet.getPacketDatagram();

        retryCounter = 1;
        while(retryCounter<=RETRY_SEND && result!=datagram.size()) {
#ifdef ENABLE_LOG4CPP
            logger.debugStream()<<getLogMessageHeader()
                               <<"Require write command "
                              <<"("<<packet.toString()<<") "
                             <<"Datagram: "<<datagram.toHex().data()<<" "
                            <<"Datagram size: "<<datagram.size()<<" "
                           <<"(Retry counter: "<<retryCounter<<")";
#endif
            waitForAck=true;
            timeoutExpired=false;

            result = socket->writeDatagram(datagram,
                                           packet.getDestinationAddress(),
                                           packet.getDestionationPort());
#ifdef ENABLE_LOG4CPP
            if(result==-1)
                logger.debugStream()<<getLogMessageHeader()
                                   <<"Command NOT writed cause error: "<<result<<" "
                                  <<"("<<packet.toString()<<"); "
                                 <<"(Retry counter: "<<retryCounter<<")";
            if(result==0)
                logger.debugStream()<<getLogMessageHeader()
                                   <<"Command writed but with 0 byte: "
                                  <<"datagram size: "<<datagram.size()<<" "
                                 <<"byte sent: "<<result<<" "
                                <<"("<<packet.toString()<<") "
                               <<"(Retry counter: "<<retryCounter<<")";
#endif
            if(result>0) {
                if(result!=datagram.size()) {
#ifdef ENABLE_LOG4CPP
                    logger.debugStream()<<getLogMessageHeader()
                                       <<"Command writed but datagram size mismatch: "
                                      <<"datagram size: "<<datagram.size()<<" "
                                     <<"byte sent: "<<result<<" "
                                    <<"("<<packet.toString()<<") "
                                   <<"(Retry counter: "<<retryCounter<<")";
#endif
                } else {
                    if(packet.isAckRequired()) {
#ifdef ENABLE_LOG4CPP
                        logger.debugStream()<<getLogMessageHeader()
                                           <<"Command writed "
                                          <<"("<<packet.toString()<<") "
                                         <<"and start waiting for ack"
                                        <<"(Retry counter: "<<retryCounter<<") "
                                       <<"(Timeout: "<<TIMEOUT_MS<<" ms)";
#endif

                        timeoutTimer->start(TIMEOUT_MS);

                        QEventLoop loop;
                        connect(this, SIGNAL(stopWaitingAck()), &loop, SLOT(quit()));
                        loop.exec();

                        timeoutTimer->stop();

                        if(timeoutExpired) {
                            result = -2;
#ifdef ENABLE_LOG4CPP
                            logger.debugStream()<<getLogMessageHeader()
                                               <<"Command write "
                                              <<"("<<packet.toString()<<") "
                                             <<"(Retry counter: "<<retryCounter<<") "
                                            <<"but ack timeout is expired";
#endif
                        }

                    }
#ifdef ENABLE_LOG4CPP
                    else {
                        logger.debugStream()<<getLogMessageHeader()
                                           <<"Command writed "
                                          <<"("<<packet.toString()<<") "
                                         <<"(Retry counter: "<<retryCounter<<") "
                                        <<"and no ack is required";
                    }
#endif
                }
            }

            retryCounter++;
        }

        datagram.clear();

        packetCache.erase(packet.getRequestId());

#ifdef ENABLE_LOG4CPP
        if(retryCounter>RETRY_SEND) {
            logger.warnStream()<<getLogMessageHeader()
                              <<"Require write cmd "
                             <<"("<<packet.toString()<<") "
                            <<"but retry counter is expired, "
                           <<"Retry counter: "<<retryCounter;
        }
#endif
    }
#ifdef ENABLE_LOG4CPP
    else {
        logger.warnStream()<<getLogMessageHeader()
                          <<"Require write cmd "
                         <<"("<<packet.toString()<<") "
                        <<"but socket is closed";
    }
#endif

    return result;
}

void UDPChannelTransmitter::fastSendCommand(AbstractPacket &packet)
{
    socket->writeDatagram(packet.getPacketDatagram(),packet.getDestinationAddress(),packet.getDestionationPort());
}

void UDPChannelTransmitter::timeoutAck()
{
    timeoutExpired=true;
    emit stopWaitingAck();
}


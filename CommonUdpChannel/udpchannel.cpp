#include "udpchannel.h"

UDPChannel::UDPChannel(QHostAddress sourceAddr,
                       quint16 sourcePort)
    : sourceAddr(sourceAddr),
      sourcePort(sourcePort)
{
    initTimeoutTimerAndLoop();

    asynchMessageEnabled = false;

#ifdef USE_LOG4CPP
    logger.infoStream()<<getLogMessageHeader()<<"New";
#endif
}

UDPChannel::UDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
                       AbstractPacketHandlerFactory *packetHandlerFactory)
    : sourceAddr(sourceAddr),
      sourcePort(sourcePort),
      packetHandlerFactory(packetHandlerFactory)
{
    initTimeoutTimerAndLoop();

    asynchMessageEnabled = true;

#ifdef USE_LOG4CPP
    logger.infoStream()<<getLogMessageHeader()<<"New";
#endif
}

UDPChannel::UDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
                       QHostAddress standardDestinationAddr, quint16 standardDestinationPort)
    : sourceAddr(sourceAddr),
      sourcePort(sourcePort),
      standardDestinationAddr(standardDestinationAddr),
      standardDestinationPort(standardDestinationPort)
{
    initTimeoutTimerAndLoop();

    asynchMessageEnabled = false;
}

UDPChannel::UDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
                       QHostAddress standardDestinationAddr, quint16 standardDestinationPort,
                       AbstractPacketHandlerFactory *packetHandlerFactory)
    : sourceAddr(sourceAddr),
      sourcePort(sourcePort),
      standardDestinationAddr(standardDestinationAddr),
      standardDestinationPort(standardDestinationPort),
      packetHandlerFactory(packetHandlerFactory)
{
    initTimeoutTimerAndLoop();

    asynchMessageEnabled = true;
}

UDPChannel::~UDPChannel()
{
    if(asynchMessageEnabled)
        delete packetHandlerFactory;
}


QHostAddress UDPChannel::getSourceAddress()
{
    return sourceAddr;
}

quint16 UDPChannel::getSourcePort()
{
    return sourcePort;
}

QHostAddress UDPChannel::getStandardDestinationAddress()
{
    return standardDestinationAddr;
}

quint16 UDPChannel::getStandardDestinationPort()
{
    return standardDestinationPort;
}


void UDPChannel::processTheDatagram(const QByteArray &datagram, QHostAddress sender, quint16 senderPort)
{
    if(waitForAck)
        manageAckMessage(datagram, sender, senderPort);

    if(asynchMessageEnabled)
        manageAsynchMessage(datagram, sender, senderPort);
}

void UDPChannel::initTimeoutTimerAndLoop()
{
    timeoutTimer = new QTimer(this);
    timeoutLoop = new QEventLoop(this);

    connect(timeoutTimer, SIGNAL(timeout()),this,SLOT(timeoutAck()));
    connect(this, SIGNAL(stopWaitingAck()), timeoutLoop, SLOT(quit()));
}

void UDPChannel::manageAckMessage(const QByteArray &datagram, QHostAddress sender, quint16 senderPort)
{
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
    if(!hasPendingDatagrams()) {
        waitForAck = false;
        emit stopWaitingAck();
    }
}

void UDPChannel::manageAsynchMessage(const QByteArray &datagram, QHostAddress sender, quint16 senderPort)
{
    quint16 handlerIdentifier = packetHandlerFactory->getPacketHandlerIdentifier(datagram);


    if(!packetHandlerFactory->isValidCode(handlerIdentifier)) {
#ifdef ENABLE_LOG4CPP
        logger.warnStream()<<getLogMessageHeader()
                          <<"Unknow handler identifier "
                         <<"Datagram: "<<datagram.toHex().data()<<" "
                        <<"Datagram size: "<<datagram.size();
#endif
        return;
    }

    AbstractPacketHandler* packetHandler = packetHandlerFactory->createPacketHandler(handlerIdentifier, datagram,
                                                                                    sender, senderPort);
    packetHandler->execute();


    if(packetHandler->isAckRequired()) { //if ack is required
        if(packetHandler->isAckAllowed()) {
            QByteArray ackDatagram = packetHandler->getAckDatagram();
            writeDatagram(ackDatagram, sender, senderPort);
#ifdef ENABLE_LOG4CPP
            logger.debugStream()<<getLogMessageHeader()
                               <<"Ack sent "
                              <<"("<<packetHandler->toString()<<") "
                             <<"Ack datagram: "<<ackDatagram.toHex().data()<<" "
                            <<"Ack datagram size: "<<ackDatagram.size();
#endif
            ackDatagram.clear();
        }
#ifdef ENABLE_LOG4CPP
        else
            logger.debugStream()<<getLogMessageHeader()
                               <<"Ack not allowed from msg handler "
                              <<"("<<packetHandler->toString()<<") "
                             <<"Datagram: "<<datagram.toHex().data()<<" "
                            <<"Datagram size: "<<datagram.size();
#endif
    }

#ifdef ENABLE_LOG4CPP
    else {
        logger.debugStream()<<getLogMessageHeader()
                           <<"Ack not required "
                          <<"("<<packetHandler->toString()<<") "
                            //<<"Datagram: "<<datagram.toHex().data()<<" "
                         <<"Datagram size: "<<datagram.size();
    }
#endif

    delete packetHandler;
}

int UDPChannel::sendPacket(AbstractPacket &packet)
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

            result = writeDatagram(datagram,
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
                        timeoutLoop->exec();
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

void UDPChannel::fastSendPacket(AbstractPacket &packet)
{
    writeDatagram(packet.getPacketDatagram(),packet.getDestinationAddress(),packet.getDestionationPort());
}

void UDPChannel::timeoutAck()
{
    timeoutExpired=true;
    emit stopWaitingAck();
}

#ifdef USE_LOG4CPP
std::string UDPChannel::getLogMessageHeader()
{
    return "Control channel ("
            + sourceAddr.toString().toStdString()
            + ":" + std::to_string((int) sourcePort) + ") - ";
}
#endif

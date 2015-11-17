#ifndef APPLICATIONCONTROLCHANNEL_H
#define APPLICATIONCONTROLCHANNEL_H

#include <QEventLoop>
#include <vector>
#include <unordered_map>

#include "CommonUdpChannel/udpchannel.h"

#include "CommonCommand/abstractcommand.h"

/**
 * @brief The ControlChannelMaster class provide control channel for the master of the GigE communication
 */
class UDPChannelTransmitter : public UDPChannel
{
    Q_OBJECT
public:
    /**
     * @brief ControlChannelMaster constructor
     * @param sourceAddr
     * @param sourcePort
     */
    UDPChannelTransmitter(QHostAddress sourceAddr, quint16 sourcePort);

    /**
     * @brief ~ControlChannelMaster deconstructor
     */
    virtual ~UDPChannelTransmitter();

    /* HIHERIT DOCS */
    void processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort);

    /**
     * @brief sendCommand a command on the channel
     * @param cmd command to send
     * @return 0 if the command is successfully sent
     */
    int sendCommand(AbstractPacket* packet);

signals:
    /**
     * @brief stopWaitingAck signal used for manage stop wait for ack
     */
    void stopWaitingAck();

public slots:
    /**
     * @brief timeoutAck this slot will fire stop waiting ack
     */
    void timeoutAck();

private:
    /**
     * @brief commandCache store command waiting for ack,
     * the key of this map if the request id of a command
     */
    std::unordered_map<int, AbstractPacket*> packetCache;

    /**
     * @brief lastReqId will incremented at new command send request
     * store last used request id
     */
    quint16 lastReqId = 0;

    /**
     * @brief retryCounter
     */
    int retryCounter;

    /**
     * @brief waitForAck
     */
    bool waitForAck;

    /**
     * @brief timeoutExpired
     */
    bool timeoutExpired;
};

#endif // APPLICATIONCONTROLCHANNEL_H

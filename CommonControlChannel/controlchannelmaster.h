#ifndef APPLICATIONCONTROLCHANNEL_H
#define APPLICATIONCONTROLCHANNEL_H

#include <QEventLoop>
#include <vector>
#include <unordered_map>

#include "CommonControlChannel/controlchannel.h"

#include "CommonMessages/abstractcommand.h"

/**
 * @brief The ControlChannelMaster class provide control channel for the master of the GigE communication
 */
class ControlChannelMaster : public ControlChannel
{
    Q_OBJECT
public:
    /**
     * @brief ControlChannelMaster constructor
     * @param sourceAddr
     * @param sourcePort
     */
    ControlChannelMaster(QHostAddress sourceAddr, quint16 sourcePort);

    /**
     * @brief ~ControlChannelMaster deconstructor
     */
    virtual ~ControlChannelMaster();

    /* HIHERIT DOCS */
    void processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort);

    /**
     * @brief sendCommand a command on the channel
     * @param cmd command to send
     * @return 0 if the command is successfully sent
     */
    int sendCommand(AbstractCommand* cmd);

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
    std::unordered_map<int, AbstractCommand*> commandCache;

    /**
     * @brief lastReqId will incremented at new command send request
     * store last used request id
     */
    int lastReqId = 0;

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

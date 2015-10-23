#ifndef APPLICATIONCONTROLCHANNEL_H
#define APPLICATIONCONTROLCHANNEL_H

#include <QEventLoop>
#include <vector>
#include <unordered_map>

#include "controlchannel.h"

#include "applicationmessagehandlerfactory.h"

#include "abstractcommand.h"

class ControlChannelMaster : public ControlChannel
{
    Q_OBJECT
public:
    ControlChannelMaster(QHostAddress sourceAddr,
                              quint16 sourcePort,
                              AbstractMessageHandlerFactory *messageHandlerFactory);

    virtual ~ControlChannelMaster();

    void processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort);

    int sendCommand(AbstractCommand* cmd);

    AbstractMessageHandler* getMessageHandler();

signals:
    void stopWaitingAck();

public slots:
    void timeoutAck();

private:

    std::unordered_map<int, AbstractCommand*> commandCache;

    int lastReqId = 0;

    int retryCounter;

    bool waitForAck;

    bool timeoutExpired;
};

#endif // APPLICATIONCONTROLCHANNEL_H

#ifndef CONTROLCHANNEL_H
#define CONTROLCHANNEL_H

#include <QUdpSocket>
#include <QHostAddress>

#include <QLoggingCategory>

#include "abstractmessagefactory.h"

/*!
 * \brief The ControlChannel class create a new control channel
 */
class ControlChannel :  public QObject
{
public:
    ControlChannel(QHostAddress sourceAddr,
                   quint16 sourcePort,
                   AbstractMessageFactory* messageHandlerFactory);

    void initSocket();

    void setMonitorAccess();

    void setCtrlAccess(QHostAddress applicationAddr, quint16 applicationPort);

    void setCtrlAccessSwitchOver(QHostAddress applicationAddr, quint16 applicationPort);

    void setExclusiveAccess(QHostAddress applicationAddr, quint16 applicationPort);

    QHostAddress getSourceAddress();

    quint16 getSourcePort();

private:

    QHostAddress sourceAddr;

    quint16 sourcePort;

    QUdpSocket* socket;

    ControlChannelPrivilege ctrlChannelPrivilege = MONITOR;

    QHostAddress applicationAddr;

    quint16 applicationPort;

    AbstractMessageFactory* messageHandlerFactory;

    void readPendingDatagrams();

    void processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort);

    bool checkChannelPrivilege(QHostAddress senderAddr, quint16 senderPort);
};

#endif // CONTROLCHANNEL_H

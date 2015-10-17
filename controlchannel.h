#ifndef CONTROLCHANNEL_H
#define CONTROLCHANNEL_H

#include <QObject>

#include <QUdpSocket>
#include <QHostAddress>

#include <QLoggingCategory>

#include "abstractmessagehandlerfactory.h"
#include "controlchannelprivilege.h"
#include "privilege.h"

#include <log4cpp/Category.hh>

/*!
 * \brief The ControlChannel class create a new control channel
 */
class ControlChannel :  public QObject
{
    Q_OBJECT
public:
    explicit ControlChannel(QObject* parent = 0);
    ControlChannel(QHostAddress sourceAddr,
                   quint16 sourcePort,
                   AbstractMessageHandlerFactory* messageHandlerFactory);

    void initSocket();

    void setMonitorAccess();

    void setCtrlAccess(QHostAddress applicationAddr, quint16 applicationPort);

    void setCtrlAccessSwitchOver(QHostAddress applicationAddr, quint16 applicationPort);

    void setExclusiveAccess(QHostAddress applicationAddr, quint16 applicationPort);

    QHostAddress getSourceAddress();

    quint16 getSourcePort();

    bool isSocketOpen();

public slots:
    void readPendingDatagrams();

protected:

    virtual void processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort) = 0;

    Privilege checkChannelPrivilege(QHostAddress senderAddr, quint16 senderPort);

    QHostAddress sourceAddr;

    quint16 sourcePort;

    QUdpSocket* socket;

    ControlChannelPrivilege ctrlChannelPrivilege = MONITOR;

    QHostAddress applicationAddr;

    quint16 applicationPort;

    AbstractMessageHandlerFactory* messageHandlerFactory;

    log4cpp::Category &logger = log4cpp::Category::getInstance( std::string("ControlChannelLog"));

    std::string getLogMessageHeader();
};

#endif // CONTROLCHANNEL_H

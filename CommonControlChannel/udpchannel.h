#ifndef CONTROLCHANNEL_H
#define CONTROLCHANNEL_H

#include <QObject>

#include <QUdpSocket>
#include <QHostAddress>

#include <QLoggingCategory>

#include <QTimer>

#include <log4cpp/Category.hh>

#include "CommonMessages/abstractmessagehandlerfactory.h"
#include "CommonControlChannel/controlchannelprivilege.h"
#include "CommonMessages/privilege.h"

/*!
 * \brief The upd channel class create a new udp socket for manage in/out datagram from the network
 */
class UDPChannel :  public QObject
{
    Q_OBJECT
public:
    /**
     * @brief ControlChannel explict constructor for QObject
     * @param parent
     */
    explicit UDPChannel(QObject* parent = 0);

    /**
     * @brief ControlChannel constructor
     * @param sourceAddr addresses that can send message on this channel
     * @param sourcePort port where this channel will be listen
     */
    UDPChannel(QHostAddress sourceAddr,
                   quint16 sourcePort);

    /**
     * @brief ~ControlChannel deconstructor
     */
    virtual ~UDPChannel();

    /**
     * @brief initSocket method create a new socket and register readPendingDatagrams method
     * for receive updates if new datagram is received
     */
    void initSocket();

    /**
     * @brief setMonitorAccess method setup the control channel level to monitor
     */
    void setMonitorAccess();

    /**
     * @brief setCtrlAccess method setup the control channel level to control access
     * @param applicationAddr
     * @param applicationPort
     */
    void setCtrlAccess(QHostAddress applicationAddr, quint16 applicationPort);

    /**
     * @brief setCtrlAccessSwitchOver method setup the control channel level to control
     * access with switch over (control can be changing with correct key)
     * @param applicationAddr
     * @param applicationPort
     */
    void setCtrlAccessSwitchOver(QHostAddress applicationAddr, quint16 applicationPort);

    /**
     * @brief setExclusiveAccess method setup the control channel level to exclusive
     * @param applicationAddr
     * @param applicationPort
     */
    void setExclusiveAccess(QHostAddress applicationAddr, quint16 applicationPort);

    /**
     * @brief getSourceAddress method
     * @return addresses that can send message on this channel
     */
    QHostAddress getSourceAddress();

    /**
     * @brief getSourcePort method
     * @return port where the channel is listen for new datagram
     */
    quint16 getSourcePort();

    /**
     * @brief isSocketOpen method
     * @return true if the socket is open
     */
    bool isSocketOpen();

public slots:
    /**
     * @brief readPendingDatagrams method will be called when
     * pending datagram are available on the socket
     */
    void readPendingDatagrams();

protected:
    /**
     * @brief processTheDatagram method
     * @param datagram received
     * @param sender ip address of the datagram sender
     * @param senderPort port where the datagram was sended
     */
    virtual void processTheDatagram(QByteArray datagram, QHostAddress sender, quint16 senderPort) = 0;

    /**
     * @brief checkChannelPrivilege method return the right of the sender
     * @param senderAddr
     * @param senderPort
     * @return right for the sender
     */
    Privilege checkChannelPrivilege(QHostAddress senderAddr, quint16 senderPort);

    /**
     * @brief TIMEOUT_MS for a sent command
     */
    int TIMEOUT_MS = 2000;

    /**
     * @brief RETRY_SEND maximum times for retry for a sent command
     */
    int RETRY_SEND = 3;

    /**
     * @brief sourceAddr ip eaddresses that can send a message to the channel
     */
    QHostAddress sourceAddr;

    /**
     * @brief sourcePort port where the is waiting for new datagram
     */
    quint16 sourcePort;

    /**
     * @brief socket
     */
    QUdpSocket* socket;

    /**
     * @brief ctrlChannelPrivilege
     */
    ControlChannelPrivilege ctrlChannelPrivilege = MONITOR;

    /**
     * @brif applicationAddr address of the application that store rights on the channel
     */
    QHostAddress applicationAddr;

    /**
     * @brief applicationPort address of the application that store rights on the channel
     */
    quint16 applicationPort;

    /**
     * @brief logger
     */
    log4cpp::Category &logger = log4cpp::Category::getInstance( std::string("ControlChannelLog"));

    /**
     * @brief getLogMessageHeader method
     * @return header for log message
     */
    std::string getLogMessageHeader();

    /**
     * @brief timeoutTimer
     */
    QTimer *timeoutTimer;
};

#endif // CONTROLCHANNEL_H

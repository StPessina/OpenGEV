#ifndef CONTROLCHANNEL_H
#define CONTROLCHANNEL_H

#include <QObject>

#include <QUdpSocket>
#include <QHostAddress>

#include <QLoggingCategory>

#include <QTimer>

#include <log4cpp/Category.hh>

#include "CommonCommand/abstractcommandhandlerfactory.h"
#include "CommonUdpChannel/controlchannelprivilege.h"

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
     *
     * this method will init SIGNAL/SLOT on udpSocket
     */
    bool initSocket();

    /**
     * @brief getSourceAddress method
     * @return addresses that can send message on this channel
     */
    virtual QHostAddress getSourceAddress() final;

    /**
     * @brief getSourcePort method
     * @return port where the channel is listen for new datagram
     */
    virtual quint16 getSourcePort() final;

    /**
     * @brief isSocketOpen method
     * @return true if the socket is open
     */
    virtual bool isSocketOpen() final;

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

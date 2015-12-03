#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#include <QThread>

#include <QTimer>
#include <QEventLoop>

#include <QHostAddress>
#include <QByteArray>

#include "CommonPacket/abstractpackethandlerfactory.h"
#include "CommonPacket/conversionutils.h"

#include <QEventLoop>
#include <vector>
#include <unordered_map>

#include "CommonCommand/abstractcommand.h"

class UDPChannel : public QThread
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
     * @brief ControlChannel constructor
     * @param sourceAddr addresses that can send message on this channel
     * @param sourcePort port where this channel will be listen
     */
    UDPChannel(QHostAddress sourceAddr,
                   quint16 sourcePort,
               AbstractPacketHandlerFactory *packetHandlerFactory);

    /**
     * @brief ~ControlChannel deconstructor
     */
    virtual ~UDPChannel();

    /**
     * @brief initSocket method create a new socket and start methods
     * for receive updates if new datagram is received
     *
     */
    virtual bool initSocket() = 0;

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
    virtual bool isSocketOpen() = 0;

    /**
     * @brief sendCommand a command on the channel
     * @param cmd command to send
     * @return 0 if the command is successfully sent
     */
    int sendPacket(AbstractPacket &packet);

    /**
     * @brief send without check, session id and ack
     * @param cmd command to send
     */
    void fastSendPacket(AbstractPacket &packet);

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

protected:

    /**
     * @brief processTheDatagram method
     * @param datagram received
     * @param sender ip address of the datagram sender
     * @param senderPort port where the datagram was sended
     */
    virtual void processTheDatagram(const QByteArray &datagram, QHostAddress sender, quint16 senderPort);

    /**
     * @brief writeDatagram method
     * @param datagram to send
     * @param destAddr destination address
     * @param destPort destination port
     * @return byte writed
     */
    virtual int writeDatagram(const QByteArray &datagram, QHostAddress destAddr, quint16 destPort) = 0;

    /**
     * @brief hasPendingDatagram
     * @return
     */
    virtual bool hasPendingDatagrams() = 0;

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

#ifdef USE_LOG4CPP
    /**
     * @brief logger
     */
    log4cpp::Category &logger = log4cpp::Category::getInstance( std::string("ControlChannelLog"));

    /**
     * @brief getLogMessageHeader method
     * @return header for log message
     */
    std::string getLogMessageHeader();
#endif

    /**
     * @brief timeoutTimer
     */
    QTimer *timeoutTimer;

    QEventLoop *timeoutLoop;

private:

    //FOR RECEIVED MESSAGE (NOT ACK)

    virtual void manageAsynchMessage(const QByteArray &datagram, QHostAddress sender, quint16 senderPort);

    bool asynchMessageEnabled;

    /**
     * @brief messageHandlerFactory store factory reference used to generate a new message handler
     */
    AbstractPacketHandlerFactory *packetHandlerFactory;

    //FOR ACK MESSAGE

    virtual void manageAckMessage(const QByteArray &datagram, QHostAddress sender, quint16 senderPort);

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

#endif // UDPSOCKET_H

#ifndef CONTROLCHANNEL_H
#define CONTROLCHANNEL_H

#include "udpchannel.h"

#include <QUdpSocket>

#ifdef USE_LOG4CPP
    #include <log4cpp/Category.hh>
#endif

/*!
 * \brief The upd channel class create a new udp socket for manage in/out datagram from the network
 */
class QtUDPChannel :  public UDPChannel
{
    Q_OBJECT
public:
    /**
     * @brief ControlChannel explict constructor for QObject
     * @param parent
     */
    explicit QtUDPChannel(QObject* parent = 0);

    /**
     * @brief ControlChannel constructor
     * @param sourceAddr addresses that can send message on this channel
     * @param sourcePort port where this channel will be listen
     */
    QtUDPChannel(QHostAddress sourceAddr,
                   quint16 sourcePort);

    /**
     * @brief ControlChannel constructor
     * @param sourceAddr addresses that can send message on this channel
     * @param sourcePort port where this channel will be listen
     */
    QtUDPChannel(QHostAddress sourceAddr,
                   quint16 sourcePort,
                 AbstractPacketHandlerFactory *packetHandlerFactory);

    /**
     * @brief ~ControlChannel deconstructor
     */
    virtual ~QtUDPChannel();

    /**
     * @brief initSocket method create a new socket and register readPendingDatagrams method
     * for receive updates if new datagram is received
     *
     * this method will init SIGNAL/SLOT on udpSocket
     */
    virtual bool initSocket() final;

    /**
     * @brief isSocketOpen method
     * @return true if the socket is open
     */
    virtual bool isSocketOpen() final;

    /**
     * @brief run method for qt thread
     */
    virtual void run();

public slots:
    /**
     * @brief readPendingDatagrams method will be called when
     * pending datagram are available on the socket
     */
    virtual void readPendingDatagrams() final;

protected:

    /**
     * @brief writeDatagram method
     * @param datagram to send
     * @param destAddr destination address
     * @param destPort destination port
     */
    virtual int writeDatagram(const QByteArray &datagram, QHostAddress destAddr, quint16 destPort);

    /**
     * @brief hasPendingDatagram
     * @return
     */
    virtual bool hasPendingDatagrams();

    /**
     * @brief socket
     */
    QUdpSocket* socket;

private:

    QByteArray datagram;
};

#endif // CONTROLCHANNEL_H

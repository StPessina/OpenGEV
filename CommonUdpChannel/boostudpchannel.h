#ifndef BOOSTUDPCHANNEL_H
#define BOOSTUDPCHANNEL_H

#include "udpchannel.h"

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

class BoostUDPChannel : public UDPChannel
{
    Q_OBJECT
public:

    /**
     * @brief ControlChannel explict constructor for QObject
     * @param parent
     */
    explicit BoostUDPChannel(QObject* parent = 0);

    /**
     * @brief BoostUdpChannel constructor
     * @param sourceAddr addresses that can send message on this channel
     * @param sourcePort port where this channel will be listen
     */
    BoostUDPChannel(QHostAddress sourceAddr,
                   quint16 sourcePort);

    /**
     * @brief BoostUdpChannel constructor
     * @param sourceAddr addresses that can send message on this channel
     * @param sourcePort port where this channel will be listen
     */
    BoostUDPChannel(QHostAddress sourceAddr,
                   quint16 sourcePort,
                 AbstractPacketHandlerFactory *packetHandlerFactory);

    /**
     * @brief BoostUDPChannel constructor
     * @param sourceAddr addresses that can send message on this channel
     * @param sourcePort port where this channel will be listen
     * @param standardDestinationAddr where this channel send as default address
     * @param standardDestinationPort where this channel send as default port
     */
    BoostUDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
               QHostAddress standardDestinationAddr, quint16 standardDestinationPort);

    /**
     * @brief BoostUDPChannel constructor
     * @param sourceAddr addresses that can send message on this channel
     * @param sourcePort port where this channel will be listen
     * @param standardDestinationAddr where this channel send as default address
     * @param standardDestinationPort where this channel send as default port
     * @param packetHandlerFactory factory for message handlers generation
     */
    BoostUDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
               QHostAddress standardDestinationAddr, quint16 standardDestinationPort,
               AbstractPacketHandlerFactory *packetHandlerFactory);

    /**
     * @brief ~BoostUdpChannel deconstructor
     */
    virtual ~BoostUDPChannel();

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

private:

  QByteArray datagram;

  void startAsynchReceive();

  void startSynchReceive();

  void handleReceive(const boost::system::error_code& error,
      std::size_t bytes_transferred);

  boost::asio::io_service io_service;

  boost::system::error_code error;

  udp::socket socket;

  int receivedBytes;

  udp::endpoint listenerEndpoint;

  udp::endpoint senderEndpoint;

  boost::array<char, 2000000> recvBuffer;
};

#endif // BOOSTUDPCHANNEL_H

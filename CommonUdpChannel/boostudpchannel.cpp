#include "boostudpchannel.h"

BoostUDPChannel::BoostUDPChannel(QHostAddress sourceAddr,
                                 quint16 sourcePort)
    : UDPChannel(sourceAddr, sourcePort),
      socket(io_service, udp::endpoint(udp::v4(), sourcePort)),
      listenerEndpoint(udp::v4(), sourcePort)
{
}

BoostUDPChannel::BoostUDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
                                 AbstractPacketHandlerFactory *packetHandlerFactory)
    : UDPChannel(sourceAddr, sourcePort, packetHandlerFactory),
      socket(io_service, udp::endpoint(udp::v4(), sourcePort)),
      listenerEndpoint(udp::v4(), sourcePort)
{
}

BoostUDPChannel::BoostUDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
                                 QHostAddress standardDestinationAddr, quint16 standardDestinationPort)
    : UDPChannel(sourceAddr, sourcePort, standardDestinationAddr, standardDestinationPort),
      socket(io_service, udp::endpoint(udp::v4(), sourcePort)),
      listenerEndpoint(udp::v4(), sourcePort)
{

}

BoostUDPChannel::BoostUDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
                           QHostAddress standardDestinationAddr, quint16 standardDestinationPort,
                           AbstractPacketHandlerFactory *packetHandlerFactory)
    : UDPChannel(sourceAddr, sourcePort, standardDestinationAddr, standardDestinationPort, packetHandlerFactory),
      socket(io_service, udp::endpoint(udp::v4(), sourcePort)),
      listenerEndpoint(udp::v4(), sourcePort)
{

}

BoostUDPChannel::~BoostUDPChannel()
{
    if(ENABLE_BOOST_ASYNCH_SOCKET==1)
        if(!io_service.stopped())
            io_service.stop();
}

bool BoostUDPChannel::initSocket()
{
    socket.set_option(boost::asio::socket_base::broadcast(true));
    if(ENABLE_BOOST_ASYNCH_SOCKET==1)
        startAsynchReceive();
    return true;
}

bool BoostUDPChannel::isSocketOpen()
{
    return true;
}

void BoostUDPChannel::run()
{
    if(ENABLE_BOOST_ASYNCH_SOCKET==1)
        while(!finished)
            io_service.run();
    else
        while(!finished)
            startSynchReceive();

    if(ENABLE_BOOST_ASYNCH_SOCKET==1)
        if(!io_service.stopped())
            io_service.stop();
}

void BoostUDPChannel::quit()
{
    finished = true;
}

int BoostUDPChannel::writeDatagram(const QByteArray &datagram, QHostAddress destAddr, quint16 destPort)
{
    udp::endpoint remoteEndpoint(boost::asio::ip::address::from_string(destAddr.toString().toStdString()),
                              ((unsigned short) destPort));

    std::size_t sentbytes = socket.send_to(boost::asio::buffer(datagram.data(),datagram.size()),
                                           remoteEndpoint);
    return sentbytes;
}

bool BoostUDPChannel::hasPendingDatagrams()
{
    return false;
}

void BoostUDPChannel::startAsynchReceive()
{
    socket.async_receive_from(
                boost::asio::buffer(recvBuffer), senderEndpoint,
                boost::bind(&BoostUDPChannel::handleReceive, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
}

void BoostUDPChannel::startSynchReceive()
{
    receivedBytes = socket.receive_from(boost::asio::buffer(recvBuffer),
              senderEndpoint, 0, error);
    if(receivedBytes>0)
        handleReceive(error, receivedBytes);
}

void BoostUDPChannel::handleReceive(const boost::system::error_code &error, std::size_t bytes_transferred)
{
    if (!error || error == boost::asio::error::message_size)
    {
        datagram.resize(bytes_transferred);
        datagram.setRawData(recvBuffer.data(),bytes_transferred);

        QHostAddress senderAddress (senderEndpoint.address().to_string().data());
        quint16 senderPort = senderEndpoint.port();

        processTheDatagram(datagram, senderAddress, senderPort);

        startAsynchReceive();
    }
}

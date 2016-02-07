#ifndef OSAPIUDPSOCKET_H
#define OSAPIUDPSOCKET_H

#include "udpchannel.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define MAXBUFLEN 4096

class OSAPIUDPChannel : public UDPChannel
{
public:
    OSAPIUDPChannel(QHostAddress sourceAddr, quint16 sourcePort);

    OSAPIUDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
                            AbstractPacketHandlerFactory *packetHandlerFactory);

    /**
     * @brief OSAPIUDPChannel constructor
     * @param sourceAddr addresses that can send message on this channel
     * @param sourcePort port where this channel will be listen
     * @param standardDestinationAddr where this channel send as default address
     * @param standardDestinationPort where this channel send as default port
     */
    OSAPIUDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
               QHostAddress standardDestinationAddr, quint16 standardDestinationPort);

    /**
     * @brief OSAPIUDPChannel constructor
     * @param sourceAddr addresses that can send message on this channel
     * @param sourcePort port where this channel will be listen
     * @param standardDestinationAddr where this channel send as default address
     * @param standardDestinationPort where this channel send as default port
     * @param packetHandlerFactory factory for message handlers generation
     */
    OSAPIUDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
               QHostAddress standardDestinationAddr, quint16 standardDestinationPort,
               AbstractPacketHandlerFactory *packetHandlerFactory);

    /**
     * @brief ~OSAPIUDPSocket deconstructor
     */
    virtual ~OSAPIUDPChannel();

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
    virtual void quit();

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

    int listenerSocket;
    struct addrinfo *pListener, *servinfoListener;

    struct sockaddr_storage their_addr;
    socklen_t addr_len;
    char buf[MAXBUFLEN];
    char s[INET6_ADDRSTRLEN];

    struct sockaddr_in destination;

    int receive();

    void* getInAddr(struct sockaddr *sa);

    quint16 getInPort(struct sockaddr *sa);

    bool finished = false;

};

#endif // OSAPIUDPSOCKET_H

#include "osapiudpchannel.h"

OSAPIUDPChannel::OSAPIUDPChannel(QHostAddress sourceAddr, quint16 sourcePort)
    : UDPChannel(sourceAddr, sourcePort)
{
}

OSAPIUDPChannel::OSAPIUDPChannel(QHostAddress sourceAddr, quint16 sourcePort, AbstractPacketHandlerFactory *packetHandlerFactory)
    : UDPChannel(sourceAddr, sourcePort, packetHandlerFactory)
{
}

OSAPIUDPChannel::~OSAPIUDPChannel()
{
    closeSocket(listenerSocket, pListener, servinfoListener);
    closeSocket(sendSocket, pSend, servinfoSend);
}

bool OSAPIUDPChannel::initSocket()
{
    listenerSocket = initSocket(NULL,
                                QString::number(sourcePort).toStdString().data(),
                                pListener,
                                servinfoListener,
                                true);

    return pListener!=NULL;
}

bool OSAPIUDPChannel::isSocketOpen()
{
    return true;
}

void OSAPIUDPChannel::run()
{
    while(true)
        receive();
}

int OSAPIUDPChannel::writeDatagram(const QByteArray &datagram, QHostAddress destAddr, quint16 destPort)
{
    if(lastSendAddress.toIPv4Address()!=destAddr.toIPv4Address() ||
            destPort!=lastSendPort) {
        closeSocket(sendSocket, pSend, servinfoSend);

        sendSocket = initSocket(destAddr.toString().toStdString().data(),
                                QString::number(destPort).toStdString().data(),
                                pSend,
                                servinfoSend,
                                false);

        if(sendSocket==-1 || pSend == NULL)
            return -1;

        if(destAddr.toIPv4Address()==QHostAddress("255.255.255.255").toIPv4Address()) {
            int broadcast = 1;
            setsockopt(sendSocket, SOL_SOCKET, SO_BROADCAST, &broadcast,
                                           sizeof broadcast);
        }

        lastSendAddress = destAddr;
        lastSendPort = destPort;
    }

    return sendto(sendSocket, datagram.data(), datagram.size(), 0,
                              pSend->ai_addr, pSend->ai_addrlen);
}

bool OSAPIUDPChannel::hasPendingDatagrams()
{
    return false;
}

int OSAPIUDPChannel::receive()
{
    int numbytes;

    addr_len = sizeof their_addr;
    if ((numbytes = recvfrom(listenerSocket, buf, MAXBUFLEN-1 , 0,
                             (struct sockaddr *)&their_addr, &addr_len)) == -1)
        return 1;

    datagram.resize(numbytes);
    datagram.setRawData(buf, numbytes);

    QHostAddress senderAddress(inet_ntop(their_addr.ss_family,
                                         getInAddr((struct sockaddr *)&their_addr),
                                         s, sizeof s));
    quint16 senderPort = getInPort((struct sockaddr *) &their_addr);

    processTheDatagram(datagram,senderAddress,senderPort);

    return numbytes;
}

int OSAPIUDPChannel::initSocket(const char *address,
                                const char *port,
                                struct addrinfo *p,
                                struct addrinfo *servinfo,
                                bool listen)
{
    struct addrinfo hints;
    int rv;

    int newSocket = -1;

    //Fill hints to 0
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC; // set to AF_INET to force IPv4
    hints.ai_socktype = SOCK_DGRAM;
    if(listen)
        hints.ai_flags = AI_PASSIVE; // use my IP

    //DNS lookup, fill hints and serverinfo (linked list to socketaddr structures)
    if ((rv = getaddrinfo(address, port, &hints, &servinfo)) != 0)
        return -1;

    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((newSocket = socket(p->ai_family, p->ai_socktype,
                             p->ai_protocol)) == -1) {
            continue;
        }

        if(listen) {
            if (bind(newSocket, p->ai_addr, p->ai_addrlen) == -1) {
                close(newSocket);
                continue;
            }
        }

        break;
    }

    if (p == NULL)
        return -1;

    return newSocket;
}

void* OSAPIUDPChannel::getInAddr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

quint16 OSAPIUDPChannel::getInPort(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return (((struct sockaddr_in*)sa)->sin_port);
    }

    return (((struct sockaddr_in6*)sa)->sin6_port);
}

void OSAPIUDPChannel::closeSocket(int socket,
                                  struct addrinfo *p,
                                  struct addrinfo *servinfo)
{
    if(socket!=-1)
        close(socket);

    freeaddrinfo(servinfo);

    if(p != NULL) {
        p = NULL;
    }


}



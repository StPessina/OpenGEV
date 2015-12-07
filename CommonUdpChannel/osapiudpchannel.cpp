#include "osapiudpchannel.h"

OSAPIUDPChannel::OSAPIUDPChannel(QHostAddress sourceAddr, quint16 sourcePort)
    : UDPChannel(sourceAddr, sourcePort)
{
}

OSAPIUDPChannel::OSAPIUDPChannel(QHostAddress sourceAddr, quint16 sourcePort, AbstractPacketHandlerFactory *packetHandlerFactory)
    : UDPChannel(sourceAddr, sourcePort, packetHandlerFactory)
{
}

OSAPIUDPChannel::OSAPIUDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
                                 QHostAddress standardDestinationAddr, quint16 standardDestinationPort)
    : UDPChannel(sourceAddr, sourcePort, standardDestinationAddr, standardDestinationPort)
{

}

OSAPIUDPChannel::OSAPIUDPChannel(QHostAddress sourceAddr, quint16 sourcePort,
                           QHostAddress standardDestinationAddr, quint16 standardDestinationPort,
                           AbstractPacketHandlerFactory *packetHandlerFactory)
    : UDPChannel(sourceAddr, sourcePort, standardDestinationAddr, standardDestinationPort, packetHandlerFactory)
{

}

OSAPIUDPChannel::~OSAPIUDPChannel()
{
    if(listenerSocket!=-1) {
        close(listenerSocket);
        listenerSocket = -1;
        freeaddrinfo(servinfoListener);
    }
}

bool OSAPIUDPChannel::initSocket()
{
    struct addrinfo hints;
    int rv;

    //Fill hints to 0
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET; // set to AF_INET to force IPv4
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE; // use my IP

    //DNS lookup, fill hints and serverinfo (linked list to socketaddr structures)
    if ((rv = getaddrinfo(NULL, QString::number(sourcePort).toStdString().data(),
                          &hints, &servinfoListener)) != 0)
        return false;

    // loop through all the results and bind to the first we can
    for(pListener = servinfoListener; pListener != NULL; pListener = pListener->ai_next) {
        if ((listenerSocket = socket(pListener->ai_family, pListener->ai_socktype,
                                     pListener->ai_protocol)) == -1) {
            continue;
        }

        if (bind(listenerSocket, pListener->ai_addr, pListener->ai_addrlen) == -1) {
            close(listenerSocket);
            continue;
        }


        break;
    }

    if (pListener == NULL)
        return false;

    //Configuration for send
    int broadcast = 1;
    setsockopt(listenerSocket, SOL_SOCKET, SO_BROADCAST, &broadcast,
               sizeof broadcast);

    memset(&destination, 0, sizeof destination);
    destination.sin_family = AF_INET;

    return true;
}

bool OSAPIUDPChannel::isSocketOpen()
{
    return listenerSocket!=-1;
}

void OSAPIUDPChannel::run()
{
    while(true)
        receive();
}

int OSAPIUDPChannel::writeDatagram(const QByteArray &datagram, QHostAddress destAddr, quint16 destPort)
{
    if(listenerSocket==-1 || pListener == NULL)
        return -1;

    inet_pton(AF_INET, destAddr.toString().toStdString().data(),
              &(destination.sin_addr));
    destination.sin_port = htons(destPort);

    return sendto(listenerSocket, datagram.data(), datagram.size(), 0,
                  (struct sockaddr*) &destination, sizeof destination);
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
        return -1;

    datagram.resize(numbytes);
    datagram.setRawData(buf, numbytes);

    QHostAddress senderAddress(inet_ntop(their_addr.ss_family,
                                         getInAddr((struct sockaddr *)&their_addr),
                                         s, sizeof s));
    quint16 senderPort = getInPort((struct sockaddr *) &their_addr);

    processTheDatagram(datagram,senderAddress,senderPort);

    return numbytes;
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
        return ntohs((((struct sockaddr_in*)sa)->sin_port));
    }

    return ntohs((((struct sockaddr_in6*)sa)->sin6_port));
}



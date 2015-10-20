#include "discoverymessagehandler.h"

DiscoveryMessageHandler::DiscoveryMessageHandler(GVDevice* target, QByteArray datagram,
                                                 QHostAddress senderAddress, quint16 senderPort)
    : AbstractMessageHandler(target, DISCOVERY_ACK, datagram, senderAddress, senderPort)
{
}

bool DiscoveryMessageHandler::isAllowed(Privilege ctrlChannelPrivilege)
{
    return true;
}

int DiscoveryMessageHandler::execute(Privilege ctrlChannelPrivilege)
{
    return 0;
}

int DiscoveryMessageHandler::getAckDatagramLengthWithoutHeader()
{
    return 246;
}

char *DiscoveryMessageHandler::getAckDatagramWithoutHeader()
{
    char* answer = new char[246];

    answer[0]=SPEC_VERSION_MINOR / 256;
    answer[1]=SPEC_VERSION_MINOR % 256;

    answer[2]=SPEC_VERSION_MAJOR / 256;
    answer[3]=SPEC_VERSION_MAJOR % 256;

    /*
    answer[4]=DEVICE MODE MSB;
    answer[5]=DEVICE MODE;
    answer[6]=DEVICE MODE;
    answer[7]=DEVICE MODE LSB;
    */

    /*
    answer[8]=RESERVED
    answer[9]=RESERVED
    */


    return new char[1];
}



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

    answer[0]=SPEC_VERSION_MAJOR / 256;
    answer[1]=SPEC_VERSION_MAJOR % 256;

    answer[2]=SPEC_VERSION_MINOR / 256;
    answer[3]=SPEC_VERSION_MINOR % 256;

    int deviceMode = ((GVDevice*)target)->getRegister(REG_DEVICE_MODE)->getValueNumb();
    answer[4]=deviceMode;
    answer[5]=deviceMode << 8;
    answer[6]=deviceMode << 16;
    answer[7]=deviceMode << 24;

    answer[8]=0; //RESERVED
    answer[9]=0; //RESERVED

    int MACHigh = ((GVDevice*)target)->getNetworkRegister(0, REG_DEVICE_MAC_ADD_HIGH)->getValueNumb();
    int MACLow = ((GVDevice*)target)->getNetworkRegister(0, REG_DEVICE_MAC_ADD_LOW)->getValueNumb();
    answer[10] = MACHigh;
    answer[11] = MACHigh << 8;
    answer[12] = MACLow;
    answer[13] = MACLow << 8;
    answer[14] = MACLow << 16;
    answer[15] = MACLow << 24;

    for (int var = 16; var < 246; ++var) {
        answer[var]=0;
    }

    return answer;
}



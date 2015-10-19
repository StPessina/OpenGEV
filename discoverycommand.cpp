#include "discoverycommand.h"

DiscoveryCommand::DiscoveryCommand(int req_id)
    : AbstractCommand(QHostAddress::Broadcast, CONTROL_CHANNEL_DEF_PORT,DISCOVERY_CMD,req_id,true, true)
{
}

DiscoveryCommand::DiscoveryCommand(QHostAddress destinationAddress, quint16 destinationPort, int req_id)
    : AbstractCommand(destinationAddress, destinationPort,DISCOVERY_CMD,req_id,true, true)
{

}

int DiscoveryCommand::getLengthWithoutHeader()
{
    return 0;
}

char *DiscoveryCommand::getCommandDatagramWithoutHeader()
{
    char* datagram = new char[0];
    return datagram;
}

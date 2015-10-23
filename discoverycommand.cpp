#include "discoverycommand.h"

DiscoveryCommand::DiscoveryCommand(GVApplication* target)
    : AbstractCommand(target, QHostAddress::Broadcast, CONTROL_CHANNEL_DEF_PORT,DISCOVERY_CMD, DISCOVERY_ACK, 0,true, true)
{
}

DiscoveryCommand::~DiscoveryCommand()
{

}

DiscoveryCommand::DiscoveryCommand(GVApplication* target, QHostAddress destinationAddress, quint16 destinationPort)
    : AbstractCommand(target, destinationAddress, destinationPort,DISCOVERY_CMD, DISCOVERY_ACK, 0,true, true)
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

int DiscoveryCommand::executeAnswer(QByteArray answer)
{
    if(checkAckHeader(answer))
        return 1;

    Device aDevice;
    QString qStringAnswer(answer);
    QString qStringAnswerNoHeader = qStringAnswer.mid(HEADER_LENGTH);

    aDevice.manufactureName = qStringAnswerNoHeader.mid(72,32);
    aDevice.modelName = qStringAnswerNoHeader.mid(104,32);
    aDevice.deviceVersion = qStringAnswerNoHeader.mid(136,32);

    aDevice.macAddress = "00:00:00:00:00:00";
    aDevice.ipAddress = QHostAddress::LocalHost;
    aDevice.subnetMask = QHostAddress::LocalHost;
    aDevice.defaultGateway = QHostAddress::LocalHost;

    dynamic_cast<GVApplication*>(target)->addDevice(aDevice);
}

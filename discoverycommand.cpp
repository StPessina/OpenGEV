#include "discoverycommand.h"

DiscoveryCommand::DiscoveryCommand(GVComponent* target)
    : AbstractCommand(target, QHostAddress::Broadcast, CONTROL_CHANNEL_DEF_PORT,DISCOVERY_CMD, DISCOVERY_ACK, 0,true, true)
{
}

DiscoveryCommand::~DiscoveryCommand()
{

}

DiscoveryCommand::DiscoveryCommand(GVComponent* target, QHostAddress destinationAddress, quint16 destinationPort)
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

    PartnerDevice aDevice;

    aDevice.manufactureName = ConversionUtils::getStringFromQByteArray(answer, 32, 80);
    aDevice.modelName = ConversionUtils::getStringFromQByteArray(answer, 32, 112);
    aDevice.deviceVersion = ConversionUtils::getStringFromQByteArray(answer, 32, 144);


    cout<<answer.at(18)<<':'
        <<answer.at(19)<<':'
        <<answer.at(20)<<':'
        <<answer.at(21)<<':'
        <<answer.at(22)<<':'
        <<answer.at(23)<<std::endl;
    char* hexAnswer = answer.toHex().data();
    cout<<hexAnswer[36] + hexAnswer[37]<<':'
                                     <<hexAnswer[38]<<hexAnswer[39]<<':'
                                     <<hexAnswer[40]<<hexAnswer[41]<<':'
                                     <<hexAnswer[42]<<hexAnswer[43]<<':'
                                     <<hexAnswer[44]<<hexAnswer[45]<<':'
                                     <<hexAnswer[46]<<hexAnswer[47]<<std::endl;
    aDevice.macAddress = QString("");

    aDevice.ipAddress = QHostAddress::LocalHost;

    aDevice.subnetMask = QHostAddress::LocalHost;

    aDevice.defaultGateway = QHostAddress::LocalHost;

    dynamic_cast<GVApplication*>(target)->addDevice(aDevice);
}

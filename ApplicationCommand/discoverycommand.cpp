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

quint16 DiscoveryCommand::getPacketBodyLength()
{
    return 0;
}

void DiscoveryCommand::appendPacketBody(QByteArray &datagram)
{
    //Nothing to append
}

int DiscoveryCommand::executeAnswer(const QByteArray &answer)
{
    this->answer = &answer;
    if(!checkAckHeader(answer))
        return getStatusCodeFromAnswer(answer);

    QByteArray answerWithoutHeader = answer.mid(8);

    PartnerDevice* aDevice = new PartnerDevice;

    //device info
    aDevice->manufactureName = ConversionUtils::getStringFromQByteArray(answerWithoutHeader, 32, 72);
    aDevice->modelName = ConversionUtils::getStringFromQByteArray(answerWithoutHeader, 32, 104);
    aDevice->deviceVersion = ConversionUtils::getStringFromQByteArray(answerWithoutHeader, 32, 136);

    //Read device mac address
    char* hexAnswer = answerWithoutHeader.toHex().data();
    char* mac = new char[17];
    int pos = 0;
    for (int i = 0; i < 6; i++) {
        mac[pos]=hexAnswer[20+i*2];
        mac[pos+1]=hexAnswer[20+i*2+1];
        if(i!=5)
            mac[pos+2]=':';
        pos+=3;
    }

    aDevice->macAddress = QString(mac);

    delete mac;

    //Ip address
    aDevice->ipAddress = QHostAddress(ConversionUtils::getIntFromQByteArray(answerWithoutHeader, 36));

    aDevice->subnetMask = QHostAddress(ConversionUtils::getIntFromQByteArray(answerWithoutHeader, 52));

    aDevice->defaultGateway = QHostAddress(ConversionUtils::getIntFromQByteArray(answerWithoutHeader, 68));

    dynamic_cast<GVApplication*>(target)->addDevice(aDevice);

    return 0;
}

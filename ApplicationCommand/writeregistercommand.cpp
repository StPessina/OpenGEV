#include "writeregistercommand.h"

WriteRegisterCommand::WriteRegisterCommand(GVComponent* target,
                                         int registerAddress,
                                         int value,
                                         QHostAddress destinationAddress,
                                         quint16 destinationPort)
    : AbstractCommand(target, destinationAddress, destinationPort, WRITEREG_CMD, WRITEREG_ACK, 0, true,false)
{
    registersData[registerAddress] = value;
}

WriteRegisterCommand::WriteRegisterCommand(GVComponent* target,
                                         std::vector<int> registerAddresses,
                                         std::vector<int> registerValues,
                                         QHostAddress destinationAddress,
                                         quint16 destinationPort)
    : AbstractCommand(target, destinationAddress, destinationPort, WRITEREG_CMD, WRITEREG_ACK, 0, true,false)
{
    int i = 0;
    foreach (int registerAddress, registerAddresses) {
        registersData[registerAddress] = registerValues.at(i);
        i++;
    }
}

WriteRegisterCommand::~WriteRegisterCommand()
{

}

quint16 WriteRegisterCommand::getPacketBodyLength()
{
    return registersData.size()*8;
}

void WriteRegisterCommand::appendPacketBody(QByteArray &datagram)
{
    //R-173c
    foreach (auto reg, registersData) {
        ConversionUtils::appendIntToQByteArray(datagram, reg.first);
        ConversionUtils::appendIntToQByteArray(datagram, reg.second);
    }
}

int WriteRegisterCommand::executeAnswer(const QByteArray &answer)
{
    this->answer = &answer;
    if(!checkAckHeader(answer))
        return 1;

    return getStatusCodeFromAnswer(answer);
}


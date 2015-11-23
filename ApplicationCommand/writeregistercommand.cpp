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

quint16 WriteRegisterCommand::getLengthWithoutHeader()
{
    return registersData.size()*8;
}

QByteArray WriteRegisterCommand::getPacketDatagramWithoutHeader()
{
    //R-173c

    QByteArray body;
    body.reserve(getLengthWithoutHeader());

    foreach (auto reg, registersData) {
        ConversionUtils::appendIntToQByteArray(&body, reg.first);
        ConversionUtils::appendIntToQByteArray(&body, reg.second);
    }

    return body;
}

int WriteRegisterCommand::executeAnswer(QByteArray answer)
{
    this->answer = answer;
    if(!checkAckHeader(answer))
        return 1;

    return getStatusCodeFromAnswer(answer);
}


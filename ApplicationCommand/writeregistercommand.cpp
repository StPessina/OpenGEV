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

char *WriteRegisterCommand::getCommandDatagramWithoutHeader()
{
    //R-173c

    char* body = new char[registersData.size()*8];

    int pointer = 0;
    foreach (auto reg, registersData) {
        ConversionUtils::setIntToCharArray(body, reg.first,pointer);
        ConversionUtils::setIntToCharArray(body, reg.second,pointer+4);
        pointer +=8;
    }
    return body;
}

int WriteRegisterCommand::executeAnswer(QByteArray answer)
{
    this->answer = answer;
    if(checkAckHeader(answer))
        return 1;

    return getStatusCode(answer);
}


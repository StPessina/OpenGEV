#include "readregistercommand.h"

ReadRegisterCommand::ReadRegisterCommand(GVComponent* target,
                                         int registerAddress,
                                         QHostAddress destinationAddress,
                                         quint16 destinationPort)
    : AbstractCommand(target, destinationAddress, destinationPort, READREG_CMD, READREG_ACK, 0, true,false)
{
    registersData[registerAddress] = 0;
}

ReadRegisterCommand::ReadRegisterCommand(GVComponent* target,
                                         std::vector<int> registerAddresses,
                                         QHostAddress destinationAddress,
                                         quint16 destinationPort)
    : AbstractCommand(target, destinationAddress, destinationPort, READREG_CMD, READREG_ACK, 0, true,false)
{
    foreach (int registerAddress, registerAddresses)
        registersData[registerAddress] = 0;
}

ReadRegisterCommand::~ReadRegisterCommand()
{

}

quint16 ReadRegisterCommand::getLengthWithoutHeader()
{
    return registersData.size()*4;
}

char *ReadRegisterCommand::getCommandDatagramWithoutHeader()
{
    //R-163c

    char* body = new char[registersData.size()*4];

    int pointer = 0;
    foreach (auto reg, registersData) {
        ConversionUtils::setIntToCharArray(body, reg.first,pointer);
        pointer +=4;
    }
    return body;
}

int ReadRegisterCommand::executeAnswer(QByteArray answer)
{
    this->answer = answer;
    if(checkAckHeader(answer))
        return 1;

    if(answer.length()-HEADER_LENGTH!=getLengthWithoutHeader())
        return 2;

    QByteArray answerWithoutHeader = answer.mid(8);

    int pointer = 0;
    foreach (auto reg, registersData) {
        registersData[reg.first] = ConversionUtils::getIntFromQByteArray(answerWithoutHeader, pointer);
        pointer += 4;
    }

    return 0;
}

int ReadRegisterCommand::getRegisterValue()
{
    auto value = registersData.begin();
    return value->second;
}

int ReadRegisterCommand::getRegisterValue(int registerAddress) {
    return registersData[registerAddress];
}

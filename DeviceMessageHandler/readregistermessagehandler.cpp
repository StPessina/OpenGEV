#include "readregistermessagehandler.h"

ReadRegisterMessageHandler::ReadRegisterMessageHandler(GVDevice *target, QByteArray datagram,
                                                       QHostAddress senderAddress, quint16 senderPort)
    : AbstractMessageHandler(target, READREG_ACK, datagram, senderAddress, senderPort)
{

}

bool ReadRegisterMessageHandler::isAllowed(Privilege ctrlChannelPrivilege)
{
    return true;
}

int ReadRegisterMessageHandler::execute(Privilege ctrlChannelPrivilege)
{
    if(!checkHeader())
        resultStatus = GEV_STATUS_INVALID_HEADER;
    else {

        QByteArray datagramWithoutHeader = datagram.mid(8);

        if(datagramWithoutHeader.size() % 4 != 0)
            resultStatus = GEV_STATUS_BAD_ALIGNMENT;
        else  {
            numberOfRegisters = datagramWithoutHeader.size()/4;
            resultStatus = GEV_STATUS_SUCCESS;
        }
    }

    return resultStatus;
}

quint16 ReadRegisterMessageHandler::getAckDatagramLengthWithoutHeader()
{
    if(resultStatus==GEV_STATUS_SUCCESS)
        return numberOfRegisters*4;
    return 0;
}

char *ReadRegisterMessageHandler::getAckDatagramWithoutHeader()
{
    if(resultStatus!=GEV_STATUS_SUCCESS)
        return NULL;

    QByteArray datagramWithoutHeader = datagram.mid(8);

    char* answer = new char[numberOfRegisters*4];

    for (int i = 0; i < numberOfRegisters*4; i+=4) {
        int regNumber = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader, i);
        int value = dynamic_cast<GVDevice*>(target)->getRegister(regNumber)->getValueNumb();
        ConversionUtils::setIntToCharArray(answer, value, i);
    }

    return answer;
}


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
            if(numberOfRegisters>0) {
                int accessibleRegisters=0;
                for (int i = 0; i < numberOfRegisters*4; i+=4) {
                    int regNumber = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader, i);
                    int access = dynamic_cast<GVDevice*>(target)->getRegister(regNumber)->getAccessType();
                    if(access==RegisterAccess::RA_WRITE)
                        break;
                    accessibleRegisters++;
                }
                if(accessibleRegisters==numberOfRegisters)
                    resultStatus = GEV_STATUS_SUCCESS;
                else
                    resultStatus = GEV_STATUS_ACCESS_DENIED; //CR-160cd
                numberOfRegisters = accessibleRegisters;
            } else
                resultStatus = GEV_STATUS_INVALID_PARAMETER;
        }
    }

    return resultStatus;
}

quint16 ReadRegisterMessageHandler::getAckDatagramLengthWithoutHeader()
{
    if(resultStatus==GEV_STATUS_SUCCESS)
        return numberOfRegisters*4;
    if(resultStatus==GEV_STATUS_ACCESS_DENIED
            && numberOfRegisters>0)
        return numberOfRegisters*4;
    return 0;
}

char *ReadRegisterMessageHandler::getAckDatagramWithoutHeader()
{
    if(resultStatus!=GEV_STATUS_SUCCESS ||
            (numberOfRegisters==0 && resultStatus==GEV_STATUS_ACCESS_DENIED))
        return NULL;

    QByteArray datagramWithoutHeader = datagram.mid(8);

    char* answer = new char[numberOfRegisters*4];

    for (int i = 0; i < numberOfRegisters*4; i+=4) { //CR-159cd
        int regNumber = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader, i);
        int value = dynamic_cast<GVDevice*>(target)->getRegister(regNumber)->getValueNumb();
        ConversionUtils::setIntToCharArray(answer, value, i);
    }

    return answer;
}


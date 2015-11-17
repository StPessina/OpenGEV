#include "readregistercommandhandler.h"

ReadRegisterCommandHandler::ReadRegisterCommandHandler(GVDevice *target, QByteArray datagram,
                                                       QHostAddress senderAddress, quint16 senderPort)
    : AbstractCommandHandler(target, READREG_ACK, datagram, senderAddress, senderPort)
{
    numberOfRegisters = 0;
}

int ReadRegisterCommandHandler::execute()
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
                    BootstrapRegister* reg = dynamic_cast<GVDevice*>(target)->getRegister(regNumber);
                    if(reg==NULL) { //CR-160cd
                        resultStatus = GEV_STATUS_INVALID_ADDRESS;
                        break;
                    }

                    int access = reg->getAccessType();
                    if(access==RegisterAccess::RA_WRITE) { //CR-160cd
                        resultStatus = GEV_STATUS_ACCESS_DENIED;
                        break;
                    }

                    accessibleRegisters++;
                }
                if(accessibleRegisters==numberOfRegisters)
                    resultStatus = GEV_STATUS_SUCCESS;


                numberOfRegisters = accessibleRegisters;
            } else
                resultStatus = GEV_STATUS_INVALID_PARAMETER;
        }
    }

    return resultStatus;
}

quint16 ReadRegisterCommandHandler::getAckDatagramLengthWithoutHeader()
{
    if(resultStatus==GEV_STATUS_SUCCESS)
        return numberOfRegisters*4;
    if(resultStatus==GEV_STATUS_ACCESS_DENIED
            && numberOfRegisters>0)
        return numberOfRegisters*4;
    return 0;
}

char *ReadRegisterCommandHandler::getAckDatagramWithoutHeader()
{
    if(resultStatus!=GEV_STATUS_SUCCESS ||
            (numberOfRegisters==0 && resultStatus==GEV_STATUS_ACCESS_DENIED))
        return NULL;

    QByteArray datagramWithoutHeader = datagram.mid(8);

    char* answer = new char[numberOfRegisters*4];

    for (int i = 0; i < numberOfRegisters*4; i+=4) { //CR-159cd
        int regNumber = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader, i);
        BootstrapRegister* reg = dynamic_cast<GVDevice*>(target)->getRegister(regNumber);
        int value = 0;
        if(reg!=NULL)
            value = reg->getValueNumb();
        ConversionUtils::setIntToCharArray(answer, value, i);
    }

    return answer;
}


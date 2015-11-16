#include "writeregistermessagehandler.h"

WriteRegisterMessageHandler::WriteRegisterMessageHandler(GVDevice *target, QByteArray datagram,
                                                       QHostAddress senderAddress, quint16 senderPort)
    : AbstractMessageHandler(target, WRITEREG_ACK, datagram, senderAddress, senderPort)
{
    numberOfRegisters = 0;
}

bool WriteRegisterMessageHandler::isAllowed(Privilege ctrlChannelPrivilege)
{
    return true;
}

int WriteRegisterMessageHandler::execute(Privilege ctrlChannelPrivilege)
{
    if(!checkHeader())
        resultStatus = GEV_STATUS_INVALID_HEADER;
    else {

        QByteArray datagramWithoutHeader = datagram.mid(8);

        if(datagramWithoutHeader.size() % 8 != 0)
            resultStatus = GEV_STATUS_BAD_ALIGNMENT;
        else  {
            numberOfRegisters = datagramWithoutHeader.size()/8;
            if(numberOfRegisters>0) {
                int accessibleRegisters=0;
                for (int i = 0; i < numberOfRegisters*8; i+=8) {
                    int regNumber = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader, i);
                    int value = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader, i+4);
                    BootstrapRegister* reg = dynamic_cast<GVDevice*>(target)->getRegister(regNumber);
                    if(reg==NULL)
                        break;
                    int access = reg->getAccessType();
                    if(access==RegisterAccess::RA_READ)
                        break;

                    reg->setValueNumb(value);

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

quint16 WriteRegisterMessageHandler::getAckDatagramLengthWithoutHeader()
{
    return 4;
}

char *WriteRegisterMessageHandler::getAckDatagramWithoutHeader()
{
    char* answer = new char[4];

    ConversionUtils::setIntToCharArray(answer,
                                       numberOfRegisters,
                                       0);

    return answer;
}


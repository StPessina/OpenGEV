#include "writeregistercommandhandler.h"

WriteRegisterCommandHandler::WriteRegisterCommandHandler(GVDevice *target, QByteArray datagram,
                                                         QHostAddress senderAddress, quint16 senderPort)
    : AbstractCommandHandler(target, WRITEREG_ACK, datagram, senderAddress, senderPort)
{
    numberOfRegisters = 0;
}

int WriteRegisterCommandHandler::execute()
{
    if(!checkHeader())
        resultStatus = GEV_STATUS_INVALID_HEADER;
    else {

        //R-171cd and R-170cd and R-172-cd
        if(dynamic_cast<GVDevice*>(target)->checkChannelPrivilege(sender,port)!=FULL)
            resultStatus = GEV_STATUS_ACCESS_DENIED;
        else {
            QByteArray datagramWithoutHeader = datagram.mid(8);

            if(datagramWithoutHeader.size() % 8 != 0)
                resultStatus = GEV_STATUS_BAD_ALIGNMENT;
            else  {
                numberOfRegisters = datagramWithoutHeader.size()/8;
                if(numberOfRegisters<=0)
                    resultStatus = GEV_STATUS_INVALID_PARAMETER;
                else {
                    int accessibleRegisters=0;
                    for (int i = 0; i < numberOfRegisters*8; i+=8) { //CR-168cd
                        int regNumber = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader, i);
                        int value = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader, i+4);
                        BootstrapRegister* reg = dynamic_cast<GVDevice*>(target)->getRegister(regNumber);
                        if(reg==NULL) { //CR-175cd
                            resultStatus = GEV_STATUS_INVALID_ADDRESS;
                            break;
                        }
                        int access = reg->getAccessType();

                        if(access==RegisterAccess::RA_READ) { //CR-175cd
                            resultStatus = GEV_STATUS_ACCESS_DENIED;
                            break;
                        }

                        switch (regNumber) {
                        case REG_CONTROL_CHANNEL_PRIVILEGE:
                            if(value==0)
                                dynamic_cast<GVDevice*>(target)->closeControlChannelPrivilege();
                            else
                                dynamic_cast<GVDevice*>(target)->changeControlChannelPrivilege(value, sender,port);
                            break;
                        default:
                            reg->setValueNumb(value);
                            break;
                        }

                        accessibleRegisters++;
                    }
                    if(accessibleRegisters==numberOfRegisters)
                        resultStatus = GEV_STATUS_SUCCESS;

                    numberOfRegisters = accessibleRegisters;
                }
            }
        }
    }

    return resultStatus;
}

quint16 WriteRegisterCommandHandler::getAckDatagramLengthWithoutHeader()
{
    return 4;
}

char *WriteRegisterCommandHandler::getAckDatagramWithoutHeader()
{
    //R-174c

    char* answer = new char[4];

    ConversionUtils::setIntToCharArray(answer,
                                       numberOfRegisters,
                                       0);

    return answer;
}


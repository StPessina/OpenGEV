#include "writeregistercommandhandler.h"

WriteRegisterCommandHandler::WriteRegisterCommandHandler(GVComponent *target, const QByteArray &receivedDatagram,
                                                         QHostAddress senderAddress, quint16 senderPort)
    : AbstractCommandHandler(target, WRITEREG_ACK, receivedDatagram, senderAddress, senderPort)
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
            QByteArray datagramWithoutHeader = receivedDatagram.mid(8);

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

                        resultStatus = dynamic_cast<GVDevice*>(target)->setRegister(regNumber, value,
                                                                                    sender, port);

                        if(resultStatus==GEV_STATUS_SUCCESS)
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

void WriteRegisterCommandHandler::appendAckDatagramWithoutHeader(QByteArray &datagram)
{
    //R-174c
    ConversionUtils::appendIntToQByteArray(datagram, numberOfRegisters);
}


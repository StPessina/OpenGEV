#include "streamimagedataleaderhandler.h"

StreamImageDataLeaderHandler::StreamImageDataLeaderHandler(GVComponent *target, QByteArray datagram,
                                                       QHostAddress senderAddress, quint16 senderPort)
    : AbstractStreamDataHandler(target, PacketFormat::DATA_LEADER_FORMAT, datagram, senderAddress, senderPort)
{
}

int StreamImageDataLeaderHandler::execute()
{
    int resultStatus;
    if(!checkHeader())
        resultStatus = GEV_STATUS_INVALID_HEADER;
    else {
        if(datagram.length()!=56) //20 Header + 36 Body for image leader package
            resultStatus = GEV_STATUS_INVALID_PARAMETER;

        quint32 pixelFormat = ConversionUtils::getIntFromQByteArray(datagram, 12);

        quint32 sizex = ConversionUtils::getIntFromQByteArray(datagram, 16);
        quint32 sizey = ConversionUtils::getIntFromQByteArray(datagram, 20);
        quint32 offsetx = ConversionUtils::getIntFromQByteArray(datagram, 24);
        quint32 offsety = ConversionUtils::getIntFromQByteArray(datagram, 28);
        quint32 paddingx = ConversionUtils::getShortFromQByteArray(datagram, 32);
        quint32 paddingy = ConversionUtils::getShortFromQByteArray(datagram, 34);

        dynamic_cast<StreamDataReceiver*>(target)->openStreamData(pixelFormat, sizex, sizey,
                                                                  offsetx, offsety,
                                                                  paddingx, paddingy);
    }

    return resultStatus;
}

quint16 StreamImageDataLeaderHandler::getAckDatagramLengthWithoutHeader()
{
    return 0;
}

char *StreamImageDataLeaderHandler::getAckDatagramWithoutHeader()
{
    return NULL;
}


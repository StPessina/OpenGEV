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

        QByteArray datagramWithoutHeader = datagram.mid(20);

        quint32 pixelFormat = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader, 12);

        quint32 sizex = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader, 16);
        quint32 sizey = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader, 20);
        quint32 offsetx = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader, 24);
        quint32 offsety = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader, 28);
        quint32 paddingx = ConversionUtils::getShortFromQByteArray(datagramWithoutHeader, 32);
        quint32 paddingy = ConversionUtils::getShortFromQByteArray(datagramWithoutHeader, 34);

        StreamDataReceiver* receiver = (StreamDataReceiver*) target;

        receiver->openStreamData(getRequestBlockId(),
                    pixelFormat, sizex, sizey,
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


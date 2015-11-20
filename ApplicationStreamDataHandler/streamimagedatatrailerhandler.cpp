#include "streamimagedatatrailerhandler.h"

StreamImageDataTrailerHandler::StreamImageDataTrailerHandler(GVComponent *target, QByteArray datagram,
                                                       QHostAddress senderAddress, quint16 senderPort)
    : AbstractStreamDataHandler(target, PacketFormat::DATA_LEADER_FORMAT, datagram, senderAddress, senderPort)
{
}

int StreamImageDataTrailerHandler::execute()
{
    int resultStatus;
    if(!checkHeader())
        resultStatus = GEV_STATUS_INVALID_HEADER;
    else {
        StreamDataReceiver* receiver = (StreamDataReceiver*) target;

        receiver->closeStreamData(getRequestBlockId(), getRequestPacketId());

    }

    return resultStatus;
}

quint16 StreamImageDataTrailerHandler::getAckDatagramLengthWithoutHeader()
{
    return 0;
}

char *StreamImageDataTrailerHandler::getAckDatagramWithoutHeader()
{
    return NULL;
}


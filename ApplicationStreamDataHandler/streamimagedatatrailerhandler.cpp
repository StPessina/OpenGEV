#include "streamimagedatatrailerhandler.h"

StreamImageDataTrailerHandler::StreamImageDataTrailerHandler(GVComponent *target, const QByteArray &receivedDatagram,
                                                       QHostAddress senderAddress, quint16 senderPort)
    : AbstractStreamDataHandler(target, PacketFormat::DATA_LEADER_FORMAT, receivedDatagram, senderAddress, senderPort)
{
}

int StreamImageDataTrailerHandler::execute()
{
    int resultStatus;
    if(!checkHeader())
        resultStatus = GEV_STATUS_INVALID_HEADER;
    else {
        StreamDataReceiver* receiver = (StreamDataReceiver*) target;

        if(!receiver->blockIdExist(getRequestBlockId()))
            return GEV_STATUS_ERROR;

        if(!isPacketResend())
            receiver->checkPacketIdSequence(getRequestBlockId(), getRequestPacketId());

        receiver->closeStreamData(getRequestBlockId(), getRequestPacketId());

        resultStatus=GEV_STATUS_SUCCESS;
    }

    return resultStatus;
}

quint16 StreamImageDataTrailerHandler::getAckDatagramLengthWithoutHeader()
{
    return 0;
}

void StreamImageDataTrailerHandler::appendAckDatagramWithoutHeader(QByteArray &datagram)
{
    //Nothing to append
}


#include "streamimagedatapayloadhandler.h"

StreamImageDataPayloadHandler::StreamImageDataPayloadHandler(GVComponent *target, const QByteArray &receivedDatagram,
                                                             QHostAddress senderAddress, quint16 senderPort)
    : AbstractStreamDataHandler(target, PacketFormat::DATA_PAYLOAD_GENIRIC_FORMAT, receivedDatagram, senderAddress, senderPort)
{
}

int StreamImageDataPayloadHandler::execute()
{
    int resultStatus;

    if(!checkHeader())
        resultStatus = GEV_STATUS_INVALID_HEADER;
    else {

        StreamDataReceiver* receiver = (StreamDataReceiver*) target;

        //quint32 pixelFormat = receiver->getPixelFormat();

        QByteArray datagramWithoutHeader = receivedDatagram.mid(20);

        const char* origin = datagramWithoutHeader.data();
        if(!receiver->addData(getRequestBlockId(), getRequestPacketId(), origin, datagramWithoutHeader.size()))
            return GEV_STATUS_ERROR;

        /*
        if(!receiver->blockIdExist(getRequestBlockId()))
            return GEV_STATUS_ERROR;

        if(!isPacketResend())
            receiver->checkPacketIdSequence(getRequestBlockId(), getRequestPacketId());

        char* origin = datagramWithoutHeader.data();
        memcpy(&((receiver->getStreamData(getRequestBlockId())->data)[datagramWithoutHeader.size()*(getRequestPacketId()-2)]),
               origin, datagramWithoutHeader.size()*sizeof(char));
        */

        resultStatus = GEV_STATUS_SUCCESS;
    }

    return resultStatus;
}

quint16 StreamImageDataPayloadHandler::getAckDatagramLengthWithoutHeader()
{
    return 0;
}

void StreamImageDataPayloadHandler::appendAckDatagramWithoutHeader(QByteArray &datagram)
{
    //Nothing to append
}


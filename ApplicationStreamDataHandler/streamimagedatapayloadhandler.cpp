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

        quint32 pixelFormat = receiver->getPixelFormat();

        QByteArray datagramWithoutHeader = receivedDatagram.mid(20);

        if(!receiver->checkNewPayload(getRequestBlockId(), getRequestPacketId()))
            return GEV_STATUS_ERROR;

        switch (pixelFormat) {
        case GVSP_PIX_MONO16: {
            /*
            int size = datagramWithoutHeader.size() / 2;
            quint16 data;
            Pixel<2> p;
            for (int i = 0; i < size; ++i) {
                data = ConversionUtils::getShortFromQByteArray(datagramWithoutHeader,i*2);
                p.pixelFormat=pixelFormat;
                p.value = data;
                receiver->addStreamData(getRequestBlockId(), getRequestPacketId(), p);
            }
            */
            char* origin = datagramWithoutHeader.data();
            memcpy(&(receiver->getStreamData(getRequestBlockId())->data)[528*(getRequestPacketId()-2)],
                   origin, datagramWithoutHeader.size()*sizeof(char));
            break;
        }
        default:
            break;
        }

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


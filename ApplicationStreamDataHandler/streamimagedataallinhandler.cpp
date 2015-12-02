#include "streamimagedataallinhandler.h"

StreamImageDataAllInHandler::StreamImageDataAllInHandler(GVComponent *target, const QByteArray &receivedDatagram,
                                                       QHostAddress senderAddress, quint16 senderPort)
    : AbstractStreamDataHandler(target, PacketFormat::DATA_ALLIN_FORMAT, receivedDatagram, senderAddress, senderPort)
{
}

int StreamImageDataAllInHandler::execute()
{
    int resultStatus;
    if(!checkHeader())
        resultStatus = GEV_STATUS_INVALID_HEADER;
    else {
        StreamImageDataLeaderHandler leaderHandler (getTarget(),
                                                    receivedDatagram.mid(0,56),
                                                    getSenderAddress(),getSenderPort());
        leaderHandler.execute();

        StreamDataReceiver* receiver = (StreamDataReceiver*) target;

        QByteArray data = receivedDatagram.mid(64);
        char* origin = data.data(); //56 leader + 8 payload
        memcpy((receiver->getStreamData(getRequestBlockId())->data),
               origin, data.size()*sizeof(char));

        receiver->closeStreamData(getRequestBlockId(), 0);
    }

    return resultStatus;
}

quint16 StreamImageDataAllInHandler::getAckDatagramLengthWithoutHeader()
{
    return 0;
}

void StreamImageDataAllInHandler::appendAckDatagramWithoutHeader(QByteArray &datagram)
{
    //Nothing to append
}


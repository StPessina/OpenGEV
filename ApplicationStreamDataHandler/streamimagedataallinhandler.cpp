#include "streamimagedataallinhandler.h"

StreamImageDataAllInHandler::StreamImageDataAllInHandler(GVComponent *target, QByteArray datagram,
                                                       QHostAddress senderAddress, quint16 senderPort)
    : AbstractStreamDataHandler(target, PacketFormat::DATA_ALLIN_FORMAT, datagram, senderAddress, senderPort)
{
}

int StreamImageDataAllInHandler::execute()
{
    int resultStatus;
    if(!checkHeader())
        resultStatus = GEV_STATUS_INVALID_HEADER;
    else {
        StreamImageDataLeaderHandler leaderHandler (getTarget(),
                                                    datagram.mid(0,56),
                                                    getSenderAddress(),getSenderPort());
        leaderHandler.execute();

        StreamDataReceiver* receiver = (StreamDataReceiver*) target;

        QByteArray data = datagram.mid(64);
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

QByteArray StreamImageDataAllInHandler::getAckDatagramWithoutHeader()
{
    QByteArray answer;
    return answer;
}


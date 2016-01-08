#include "packetresendcommandhandler.h"

PacketResendCommandHandler::PacketResendCommandHandler(GVComponent *target, const QByteArray &receivedDatagram,
                                                       QHostAddress senderAddress, quint16 senderPort)
    : AbstractCommandHandler(target, PACKETRESEND_ACK, receivedDatagram, senderAddress, senderPort)
{
}

int PacketResendCommandHandler::execute()
{
    if(!checkHeader())
        resultStatus = GEV_STATUS_INVALID_HEADER;
    else {
        //CR-199cd No permission check

        QByteArray datagramWithoutHeader = receivedDatagram.mid(8);

        GVDevice* device = (GVDevice*) target;

        quint16 channelId = ConversionUtils::getShortFromQByteArray(datagramWithoutHeader,0);

        StreamChannelTransmitter* transmitter = device->getStreamChannel(channelId);

        quint32 firstPacketId = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader,4);
        quint32 lastPacketId = ConversionUtils::getIntFromQByteArray(datagramWithoutHeader,8);

        quint64 blockId64 = ConversionUtils::getLongFromQByteArray(datagramWithoutHeader, 12);

        for (int i = firstPacketId; i <= lastPacketId; ++i)
            transmitter->insertPacketResendInIncomingData(blockId64, i);

        resultStatus = GEV_STATUS_SUCCESS;
    }

    return resultStatus;
}

quint16 PacketResendCommandHandler::getAckBodyLength()
{
    return 0; //Ack is directly on stream channel
}

void PacketResendCommandHandler::appendAckBody(QByteArray &datagram)
{
    //Nothing to append
}


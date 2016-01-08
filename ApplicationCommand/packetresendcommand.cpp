#include "packetresendcommand.h"

PacketResendCommand::PacketResendCommand(GVComponent* target,
                                         QHostAddress destinationAddress,
                                         quint16 destinationPort,
                                         quint16 streamChannelNr,
                                         quint64 blockId,
                                         quint32 firstPacketId, quint32 lastPacketId)
    : AbstractCommand(target, destinationAddress, destinationPort, PACKETRESEND_CMD, PACKETRESEND_ACK, 0, false,false),
      streamChannelNr(streamChannelNr),
      blockId(blockId),
      firstPacketId(firstPacketId),
      lastPacketId(lastPacketId)
{

}

PacketResendCommand::~PacketResendCommand()
{

}

quint16 PacketResendCommand::getPacketBodyLength()
{
    return 20;
}

void PacketResendCommand::appendPacketBody(QByteArray &datagram)
{
    //O-166cd
    ConversionUtils::appendShortToQByteArray(datagram, streamChannelNr);
    ConversionUtils::appendShortToQByteArray(datagram, 0); //Block id/reserved
    ConversionUtils::appendIntToQByteArray(datagram, firstPacketId);
    ConversionUtils::appendIntToQByteArray(datagram, lastPacketId);
    ConversionUtils::appendLongToQByteArray(datagram, blockId);
}

int PacketResendCommand::executeAnswer(const QByteArray &answer)
{
    return 0;
}

short PacketResendCommand::getHeaderFlag()
{
    //Redefine header flag for EI enable
    //(3° bit = 1)
    return 0x10;
}

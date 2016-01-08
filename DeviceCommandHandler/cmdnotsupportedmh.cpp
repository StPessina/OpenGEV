#include "cmdnotsupportedmh.h"

CmdNotSupportedMH::CmdNotSupportedMH(GVComponent* target, quint16 cmdCode,
                                     const QByteArray &receivedDatagram,
                                     QHostAddress senderAddress,quint16 senderPort)
    : AbstractCommandHandler(target,
      cmdCode,
      receivedDatagram,
      senderAddress,
      senderPort)
{
    this->resultStatus = GEV_STATUS_NOT_IMPLEMENTED;
}

CmdNotSupportedMH::~CmdNotSupportedMH()
{

}

int CmdNotSupportedMH::execute()
{
    return -1;
}

void CmdNotSupportedMH::appendAckBody(QByteArray &datagram)
{
    //Nothing to append
}

quint16 CmdNotSupportedMH::getAckBodyLength()
{
    return 0;
}

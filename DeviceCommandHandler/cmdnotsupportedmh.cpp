#include "cmdnotsupportedmh.h"

CmdNotSupportedMH::CmdNotSupportedMH(GVComponent* target, quint16 cmdCode, QByteArray datagram, QHostAddress senderAddress,quint16 senderPort)
    : AbstractCommandHandler(target,
      cmdCode,
      datagram,
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

char *CmdNotSupportedMH::getAckDatagramWithoutHeader()
{
    return NULL;
}

quint16 CmdNotSupportedMH::getAckDatagramLengthWithoutHeader()
{
    return 0;
}

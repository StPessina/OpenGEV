#include "cmdnotsupportedmh.h"

CmdNotSupportedMH::CmdNotSupportedMH(GVComponent* target, int cmdCode, QByteArray datagram, QHostAddress senderAddress,quint16 senderPort)
    : AbstractMessageHandler(target,
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

bool CmdNotSupportedMH::isAllowed(Privilege ctrlChannelPrivilege)
{
    return true;
}

int CmdNotSupportedMH::execute(Privilege ctrlChannelPrivilege)
{
    return -1;
}

char *CmdNotSupportedMH::getAckDatagramWithoutHeader()
{
    return new char[1];
}

int CmdNotSupportedMH::getAckDatagramLengthWithoutHeader()
{
    return 0;
}

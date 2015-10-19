#ifndef CMDNOTSUPPORTEDMH_H
#define CMDNOTSUPPORTEDMH_H

#include "abstractmessagehandler.h"

#include "deviceackstatus.h"

class CmdNotSupportedMH : public AbstractMessageHandler
{
public:
    CmdNotSupportedMH(GVComponent* target, int msgCode, QByteArray datagram, QHostAddress senderAddress,quint16 senderPort);

    bool isAllowed(Privilege ctrlChannelPrivilege);

    int execute(Privilege ctrlChannelPrivilege);

protected:
    char* getAckDatagramWithoutHeader();

    int getAckDatagramLengthWithoutHeader();
};

#endif // CMDNOTSUPPORTEDMH_H

#ifndef READREGISTERMESSAGEHANDLER_H
#define READREGISTERMESSAGEHANDLER_H

#include "Device/deviceregisters.h"

#include "CommonMessages/abstractmessagehandler.h"
#include "Device/gvdevice.h"

#include "CommonMessages/conversionutils.h"

#include "DeviceMessageHandler/deviceackcode.h"
#include "DeviceMessageHandler/deviceackstatus.h"

#include "opengv_global.h"


class ReadRegisterMessageHandler : public AbstractMessageHandler
{
public:
    ReadRegisterMessageHandler(GVDevice* target,
                               QByteArray datagram,
                               QHostAddress senderAddress,
                               quint16 senderPort);

    bool isAllowed(Privilege ctrlChannelPrivilege);

    int execute(Privilege ctrlChannelPrivilege);

    quint16 getAckDatagramLengthWithoutHeader();

    char* getAckDatagramWithoutHeader();

private:
    int numberOfRegisters;
};

#endif // READREGISTERMESSAGEHANDLER_H

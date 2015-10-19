#ifndef DEVICEDISCOVERYMESSAGEHANDLER_H
#define DEVICEDISCOVERYMESSAGEHANDLER_H

#include <QtNetwork/QNetworkInterface>

#include "abstractmessagehandler.h"
#include "gvdevice.h"

#include "deviceackcode.h"

#include "opengv_global.h"

class DiscoveryMessageHandler : public AbstractMessageHandler
{
public:
    DiscoveryMessageHandler(GVDevice* target,
                            QByteArray datagram,
                            QHostAddress senderAddress,
                            quint16 senderPort);

    bool isAllowed(Privilege ctrlChannelPrivilege);

    int execute(Privilege ctrlChannelPrivilege);

    int getAckDatagramLengthWithoutHeader();

    char* getAckDatagramWithoutHeader();
};

#endif // DEVICEDISCOVERYMESSAGEHANDLER_H

#ifndef DEVICEDISCOVERYMESSAGEHANDLER_H
#define DEVICEDISCOVERYMESSAGEHANDLER_H

#include <QtNetwork/QNetworkInterface>

#include "Device/deviceregisters.h"

#include "CommonMessages/abstractmessagehandler.h"
#include "Device/gvdevice.h"

#include "CommonMessages/conversionutils.h"

#include "DeviceMessageHandler/deviceackcode.h"
#include "DeviceMessageHandler/deviceackstatus.h"

#include "opengv_global.h"

/**
 * @brief The DiscoveryMessageHandler class answer to discovery command from application
 */
class DiscoveryMessageHandler : public AbstractMessageHandler
{
public:
    DiscoveryMessageHandler(GVDevice* target,
                            QByteArray datagram,
                            QHostAddress senderAddress,
                            quint16 senderPort);

    bool isAllowed(Privilege ctrlChannelPrivilege);

    int execute(Privilege ctrlChannelPrivilege);

    quint16 getAckDatagramLengthWithoutHeader();

    char* getAckDatagramWithoutHeader();
};

#endif // DEVICEDISCOVERYMESSAGEHANDLER_H

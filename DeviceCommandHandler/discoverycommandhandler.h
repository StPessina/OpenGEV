#ifndef DEVICEDISCOVERYMESSAGEHANDLER_H
#define DEVICEDISCOVERYMESSAGEHANDLER_H

#include <QtNetwork/QNetworkInterface>

#include "Device/deviceregisters.h"

#include "CommonCommand/abstractcommandhandler.h"
#include "Device/gvdevice.h"

#include "CommonPacket/conversionutils.h"

#include "DeviceCommandHandler/deviceackcode.h"

#include "opengev_global.h"

/**
 * @brief The DiscoveryMessageHandler class answer to discovery command from application
 */
class DiscoveryCommandHandler : public AbstractCommandHandler
{
public:
    DiscoveryCommandHandler(GVComponent* target,
                            const QByteArray &receivedDatagram,
                            QHostAddress senderAddress,
                            quint16 senderPort);

    virtual int execute();

    virtual quint16 getAckDatagramLengthWithoutHeader();

    virtual void appendAckDatagramWithoutHeader(QByteArray &datagram);
};

#endif // DEVICEDISCOVERYMESSAGEHANDLER_H

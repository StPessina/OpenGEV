#ifndef PACKETRESENDCOMMANDHANDLER_H
#define PACKETRESENDCOMMANDHANDLER_H

#include "Device/deviceregisters.h"
#include "CommonBootstrapRegister/bootstrapregister.h"

#include "CommonCommand/abstractcommandhandler.h"
#include "Device/gvdevice.h"

#include "CommonPacket/conversionutils.h"

#include "DeviceCommandHandler/deviceackcode.h"

#include "Device/gvdevice.h"

#include "opengv_global.h"

/**
 * @brief The PacketResendCommandHandler class implements read register handler
 * required (R-157cd)
 */
class PacketResendCommandHandler : public AbstractCommandHandler
{
public:
    PacketResendCommandHandler(GVComponent* target,
                               const QByteArray &receivedDatagram,
                               QHostAddress senderAddress,
                               quint16 senderPort);

    int execute();

protected:

    quint16 getAckDatagramLengthWithoutHeader();

    /**
     * @brief getAckDatagramWithoutHeader
     * @return datagram (R-164c)
     */
    void appendAckDatagramWithoutHeader(QByteArray &datagram);

};

#endif // READREGISTERMESSAGEHANDLER_H

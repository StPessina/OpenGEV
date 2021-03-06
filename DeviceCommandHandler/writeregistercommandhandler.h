#ifndef WRITEREGISTERMESSAGEHANDLER_H
#define WRITEREGISTERMESSAGEHANDLER_H

#include "Device/deviceregisters.h"
#include "CommonBootstrapRegister/bootstrapregister.h"

#include "CommonCommand/abstractcommandhandler.h"
#include "Device/gvdevice.h"

#include "CommonPacket/conversionutils.h"

#include "DeviceCommandHandler/deviceackcode.h"

#include "opengev_global.h"

/**
 * @brief The WriteRegisterMessageHandler class implements ack for write message channel
 * required R-174c
 */
class WriteRegisterCommandHandler : public AbstractCommandHandler
{
public:
    WriteRegisterCommandHandler(GVComponent* target,
                               const QByteArray &receivedDatagram,
                               QHostAddress senderAddress,
                               quint16 senderPort);

    int execute();

protected:

    quint16 getAckBodyLength();

    /**
     * @brief getAckDatagramWithoutHeader
     * @return datagram (R-164c)
     */
    void appendAckBody(QByteArray &datagram);

private:
    int numberOfRegisters;
};

#endif // WRITEREGISTERMESSAGEHANDLER_H

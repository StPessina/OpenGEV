#ifndef WRITEREGISTERMESSAGEHANDLER_H
#define WRITEREGISTERMESSAGEHANDLER_H

#include "Device/deviceregisters.h"
#include "CommonBootstrapRegister/bootstrapregister.h"

#include "CommonCommand/abstractcommandhandler.h"
#include "Device/gvdevice.h"

#include "CommonPacket/conversionutils.h"

#include "DeviceCommandHandler/deviceackcode.h"

#include "opengv_global.h"

/**
 * @brief The WriteRegisterMessageHandler class implements ack for write message channel
 * required R-174c
 */
class WriteRegisterCommandHandler : public AbstractCommandHandler
{
public:
    WriteRegisterCommandHandler(GVComponent* target,
                               QByteArray datagram,
                               QHostAddress senderAddress,
                               quint16 senderPort);

    int execute();

    quint16 getAckDatagramLengthWithoutHeader();

    /**
     * @brief getAckDatagramWithoutHeader
     * @return datagram (R-164c)
     */
    QByteArray getAckDatagramWithoutHeader();

private:
    int numberOfRegisters;
};

#endif // WRITEREGISTERMESSAGEHANDLER_H

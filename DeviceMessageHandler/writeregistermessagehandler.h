#ifndef WRITEREGISTERMESSAGEHANDLER_H
#define WRITEREGISTERMESSAGEHANDLER_H

#include "Device/deviceregisters.h"
#include "CommonBootstrapRegister/bootstrapregister.h"

#include "CommonMessages/abstractmessagehandler.h"
#include "Device/gvdevice.h"

#include "CommonMessages/conversionutils.h"

#include "DeviceMessageHandler/deviceackcode.h"
#include "DeviceMessageHandler/deviceackstatus.h"

#include "opengv_global.h"

/**
 * @brief The WriteRegisterMessageHandler class implements ack for write message channel
 * required R-174c
 */
class WriteRegisterMessageHandler : public AbstractMessageHandler
{
public:
    WriteRegisterMessageHandler(GVDevice* target,
                               QByteArray datagram,
                               QHostAddress senderAddress,
                               quint16 senderPort);

    bool isAllowed(Privilege ctrlChannelPrivilege);

    int execute(Privilege ctrlChannelPrivilege);

    quint16 getAckDatagramLengthWithoutHeader();

    /**
     * @brief getAckDatagramWithoutHeader
     * @return datagram (R-164c)
     */
    char* getAckDatagramWithoutHeader();

private:
    int numberOfRegisters;
};

#endif // WRITEREGISTERMESSAGEHANDLER_H

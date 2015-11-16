#ifndef READREGISTERMESSAGEHANDLER_H
#define READREGISTERMESSAGEHANDLER_H

#include "Device/deviceregisters.h"

#include "CommonMessages/abstractmessagehandler.h"
#include "Device/gvdevice.h"

#include "CommonMessages/conversionutils.h"

#include "DeviceMessageHandler/deviceackcode.h"
#include "DeviceMessageHandler/deviceackstatus.h"

#include "opengv_global.h"

/**
 * @brief The ReadRegisterMessageHandler class implements read register handler
 * required (R-157cd)
 */
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

    /**
     * @brief getAckDatagramWithoutHeader
     * @return datagram (R-164c)
     */
    char* getAckDatagramWithoutHeader();

private:
    int numberOfRegisters;
};

#endif // READREGISTERMESSAGEHANDLER_H

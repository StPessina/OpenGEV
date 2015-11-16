#ifndef CMDNOTSUPPORTEDMH_H
#define CMDNOTSUPPORTEDMH_H

#include "CommonMessages/abstractmessagehandler.h"

#include "DeviceMessageHandler/deviceackstatus.h"

/**
 * @brief The CmdNotSupportedMH class is special handler for not definied command code
 */
class CmdNotSupportedMH : public AbstractMessageHandler
{
public:
    /**
     * @brief CmdNotSupportedMH constructor
     * @param target
     * @param msgCode
     * @param datagram
     * @param senderAddress
     * @param senderPort
     */
    CmdNotSupportedMH(GVComponent* target, quint16 msgCode,
                      QByteArray datagram, QHostAddress senderAddress,quint16 senderPort);

    /**
     * @brief ~CmdNotSupportedMH deconstructor
     */
    virtual ~CmdNotSupportedMH();

    bool isAllowed(Privilege ctrlChannelPrivilege);

    int execute(Privilege ctrlChannelPrivilege);

protected:
    char* getAckDatagramWithoutHeader();

    quint16 getAckDatagramLengthWithoutHeader();
};

#endif // CMDNOTSUPPORTEDMH_H

#ifndef CMDNOTSUPPORTEDMH_H
#define CMDNOTSUPPORTEDMH_H

#include "CommonCommand/abstractcommandhandler.h"

#include "DeviceCommandHandler/deviceackstatus.h"

/**
 * @brief The CmdNotSupportedMH class is special handler for not definied command code
 */
class CmdNotSupportedMH : public AbstractCommandHandler
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

    int execute();

protected:
    char* getAckDatagramWithoutHeader();

    quint16 getAckDatagramLengthWithoutHeader();
};

#endif // CMDNOTSUPPORTEDMH_H

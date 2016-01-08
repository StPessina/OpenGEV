#ifndef CMDNOTSUPPORTEDMH_H
#define CMDNOTSUPPORTEDMH_H

#include "CommonCommand/abstractcommandhandler.h"

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
                      const QByteArray &receivedDatagram,
                      QHostAddress senderAddress,quint16 senderPort);

    /**
     * @brief ~CmdNotSupportedMH deconstructor
     */
    virtual ~CmdNotSupportedMH();

    int execute();

protected:
    void appendAckBody(QByteArray &datagram);

    quint16 getAckBodyLength();
};

#endif // CMDNOTSUPPORTEDMH_H

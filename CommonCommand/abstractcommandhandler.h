#ifndef ABSTRACTCOMMANDHANDLER_H
#define ABSTRACTCOMMANDHANDLER_H

#include <QString>
#include <QHostAddress>

#include "CommonComponent/gvcomponent.h"
#include "CommonUdpChannel/privilege.h"

#include "CommonPacket/abstractpackethandler.h"

#include "CommonPacket/status.h"

/**
 * \brief The AbstractCommandHandler class is generic handler
 * for incoming command
 */
class AbstractCommandHandler : public AbstractPacketHandler
{
public:
    /**
     * @brief AbstractCommandHandler
     * @param target component where the new message should be applied
     * @param ackCode acknoledgment code
     * @param datagram datagram received
     * @param senderAddress who send the datagram
     * @param senderPort destination port of the datagram
     */
    AbstractCommandHandler(GVComponent* target,
                           quint16 ackCode,
                           const QByteArray &receivedDatagram,
                           QHostAddress senderAddress,
                           quint16 senderPort);


    /**
     * @brief ~AbstractCommandHandler decostructor
     */
    virtual ~AbstractCommandHandler();


    virtual bool isAckRequired() final;

    /**
     * @brief readRequestCommandCode extract command code from a datagram
     * @param datagram
     * @return command code
     */
    static quint16 readRequestCommandCode(const QByteArray &datagram);

    /**
     * @brief readRequestRequestLength extract length from a datagram
     * @param datagram
     * @return length
     */
    static quint16 readRequestLength(const QByteArray &datagram);

    /**
     * @brief readRequestRequestId extract request id from a datagram
     * @param datagram
     * @return request id
     */
    static quint16 readRequestRequestId(const QByteArray &datagram);

    /**
     * @brief getRequestCommandCode
     * @return request command code
     */
    virtual quint16 getRequestCommandCode() final;

    /**
     * @brief getRequestLength
     * @return request length
     */
    virtual quint16 getRequestLength() final;

    /**
     * @brief getRequestId
     * @return request id
     */
    virtual quint16 getRequestId() final;

    /**
     * @brief getResultStatus
     * @return result of execution
     */
    virtual Status getResultStatus() final;

    /**
     * @brief toString
     * @return handler major info as string
     */
    virtual std::string toString();

protected:

    quint16 ackCode;

    Status resultStatus;

    quint16 requestCommandCode;

    quint16 requestLength;

    quint16 reqId;

    quint16 getAckHeaderLength() final;

    /**
     * @brief getAckHeader
     * @return header for ack message
     */
    virtual void appendAckHeader(QByteArray &datagram) final;

    /**
     * @brief checkHeader check received datagram
     * @return true if header is good
     */
    virtual bool checkHeader() final;
};

#endif // ABSTRACTCOMMANDHANDLER_H

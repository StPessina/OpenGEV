#ifndef ABSTRACTMESSAGEHANDLER_H
#define ABSTRACTMESSAGEHANDLER_H

#include <QString>
#include <QHostAddress>

#include "gvcomponent.h"
#include "privilege.h"

/*!
 * \brief The AbstractMessageHandler class
 */
class AbstractMessageHandler
{
public:
    /*!
     * \brief AbstractMessage
     * \param Target of this command
     */
    AbstractMessageHandler(GVComponent* target,
                           int ackCode,
                           QByteArray datagram,
                           QHostAddress senderAddress,
                           quint16 senderPort);


    virtual ~AbstractMessageHandler();

    /*!
     * \brief getTarget
     * \return target for this command
     */
    GVComponent* getTarget();

    QByteArray* getDatagram();

    QHostAddress getSenderAddress();

    quint16 getSenderPort();

    /*!
     * \brief isAllowed
     * \return true if this command is allowed on the target
     */
    virtual bool isAllowed(Privilege ctrlChannelPrivilege) = 0;

    /*!
     * \brief execute a command from
     * \param datagram data received
     * \param sender of datagram
     * \param port
     * \return error code
     */
    virtual int execute(Privilege ctrlChannelPrivilege) = 0;

    bool isAckAllowed();

    static int readRequestCommandCode(QByteArray* datagram);

    static int readRequestRequestLength(QByteArray* datagram);

    static int readRequestRequestId(QByteArray* datagram);

    int getRequestCommandCode();

    int getRequestLength();

    int getRequestId();

    int getResultStatus();

    /*!
     * \brief getAck
     * \return message for acknowledgement
     */
    virtual QByteArray* getAckDatagram() final;

    std::string toString();

protected:
    /*!
     * \brief target component
     */
    GVComponent* target;

    QByteArray datagram;

    QHostAddress sender;

    quint16 port;

    int ackCode;

    int resultStatus;

    int requestCommandCode;

    int requestLength;

    int reqId;

    bool ackNotAllowed = false;

    virtual char* getAckHeader() final;

    virtual bool checkHeader() final;

    /*!
     * \brief getAck
     * \return message for acknowledgement
     */
    virtual char* getAckDatagramWithoutHeader() = 0;

    /*!
     * \brief getAck message without length
     * \return message for acknowledgement
     */
    virtual int getAckDatagramLengthWithoutHeader() = 0;

};

#endif // ABSTRACTMESSAGEHANDLER_H

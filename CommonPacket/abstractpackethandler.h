#ifndef ABSTRACTPACKETHANDLER_H
#define ABSTRACTPACKETHANDLER_H

#include <QString>
#include <QHostAddress>

#include "CommonComponent/gvcomponent.h"
#include "CommonUdpChannel/privilege.h"

#include "CommonPacket/conversionutils.h"

/**
 * \brief The AbstractMessageHandler class is generic handler
 * for incoming message on slave channel
 */
class AbstractPacketHandler
{
public:
    /**
     * @brief AbstractPacketHandler
     * @param target component where the new message should be applied
     * @param datagram datagram received
     * @param senderAddress who send the datagram
     * @param senderPort destination port of the datagram
     */
    AbstractPacketHandler(GVComponent* target,
                           const QByteArray &receivedDatagram,
                           QHostAddress senderAddress,
                           quint16 senderPort);


    /**
     * @brief ~AbstractMessageHandler decostructor
     */
    virtual ~AbstractPacketHandler();

    /**
     * @brief getTarget
     * @return target for this command
     */
    virtual GVComponent* getTarget() final;

    /**
     * @brief getDatagram
     * @return datagram received
     */
    virtual const QByteArray &getReceivedDatagram() final;

    /**
     * @brief getSenderAddress
     * @return sender address
     */
    virtual QHostAddress getSenderAddress() final;


    virtual quint16 getSenderPort() final;

    /**
     * @brief execute a command on the target
     * @return error code
     */
    virtual int execute() = 0;

    /**
     * @brief isAckRequired
     * @return true if ack is required from sender
     */
    virtual bool isAckRequired() = 0;

    /**
     * @brief isAckAllowed check if the command is allowed on the target
     * @return true if it's allowed
     */
    virtual bool isAckAllowed() final;


    /**
     * @brief getAck
     * @return message for acknowledgement
     */
    virtual const QByteArray &getAckDatagram() final;

    /**
     * @brief toString
     * @return handler major info as string
     */
    virtual std::string toString();

protected:
    /*!
     * \brief target component
     */
    GVComponent* target;

    const QByteArray &receivedDatagram;

    QHostAddress sender;

    quint16 port;

    bool ackNotAllowed = false;

    QByteArray ackDatagram;

    /**
     * @brief getLengthWithoutHeader
     * @return length of the command request
     */
    virtual quint16 getAckHeaderLength() = 0;

    /**
     * @brief getAckHeader
     * @return header for ack message
     */
    virtual void appendAckHeader(QByteArray &datagram) = 0;

    /*!
     * \brief getAck
     * \return message for acknowledgement without header
     */
    virtual void appendAckDatagramWithoutHeader(QByteArray &datagram) = 0;

    /*!
     * \brief getAck message without length
     * \return message for acknowledgement
     */
    virtual quint16 getAckDatagramLengthWithoutHeader() = 0;

};

#endif // ABSTRACTPACKETHANDLER_H

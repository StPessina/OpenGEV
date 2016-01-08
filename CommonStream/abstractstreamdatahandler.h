#ifndef ABSTRACTSTREAMDATAHANDLER_H
#define ABSTRACTSTREAMDATAHANDLER_H

#include <QString>
#include <QHostAddress>

#include "CommonComponent/gvcomponent.h"
#include "CommonUdpChannel/privilege.h"

#include "CommonPacket/abstractpackethandler.h"

#include "CommonPacket/status.h"

#include "CommonStream/packetformat.h"

/**
 * \brief The AbstractStreamDataHandler class is generic handler
 * for incoming stream packet handler. No ack message is required for
 * this kind of message (R-164c).
 */
class AbstractStreamDataHandler : public AbstractPacketHandler
{
public:
    /**
     * @brief AbstractStreamDataHandler
     * @param target is the stream channel receiver where the stream data will be stored
     * @param packetFormat packet format
     * @param datagram datagram received
     * @param senderAddress who send the datagram
     * @param senderPort destination port of the datagram
     */
    AbstractStreamDataHandler(GVComponent* target,
                           quint32 packetFormat,
                           const QByteArray &datagram,
                           QHostAddress senderAddress,
                           quint16 senderPort);


    /**
     * @brief ~AbstractStreamDataHandler decostructor
     */
    virtual ~AbstractStreamDataHandler();


    virtual bool isAckRequired() final;

    /**
     * @brief readRequestPacketFormat extract packetformat from a datagram
     * @param datagram
     * @return command code
     */
    static quint32 readRequestPacketFormat(const QByteArray &datagram);

    /**
     * @brief readRequestPacketId extract packetid from a datagram
     * @param datagram
     * @return packetid
     */
    static quint32 readRequestPacketId(const QByteArray &datagram);

    /**
     * @brief readRequestBlockId extract block id from a datagram
     * @param datagram
     * @return block id
     */
    static quint64 readRequestBlockId(const QByteArray &datagram);

    /**
     * @brief getRequestPacketFormat
     * @return request packet format
     */
    virtual quint32 getRequestPacketFormat() final;

    /**
     * @brief getRequestPacketId
     * @return request packet id
     */
    virtual quint32 getRequestPacketId() final;

    /**
     * @brief getRequestBlockId
     * @return request block id
     */
    virtual quint64 getRequestBlockId() final;

    virtual bool isPacketResend() final;

    /**
     * @brief toString
     * @return handler major info as string
     */
    virtual std::string toString();

protected:

    /**
     * @brief requestPacketFormat
     */
    quint32 requestPacketFormat;

    /**
     * @brief requestBlockId
     */
    quint64 requestBlockId;

    /**
     * @brief requestPacketId
     */
    quint32 requestPacketId;

    /**
     * @brief getAckHeaderLength
     * @return 0 always (R-164c no ack required for stream message)
     */
    quint16 getAckHeaderLength() final;

    /**
     * @brief getAckHeader
     * @return datagram (R-164c no ack required for stream message)
     */
    virtual void appendAckHeader(QByteArray &datagram) final;

    /**
     * @brief checkHeader check received datagram
     * @return true if header is good
     */
    virtual bool checkHeader() final;

    /**
     * @brief getAckBodyLength
     * @return 0 always (R-164c no ack required for stream message)
     */
    virtual quint16 getAckBodyLength() final;

    /**
     * @brief appendAckBody
     * @param datagram (R-164c no ack required for stream message)
     */
    virtual void appendAckBody(QByteArray &datagram) final;
};

#endif // ABSTRACTSTREAMDATAHANDLER_H

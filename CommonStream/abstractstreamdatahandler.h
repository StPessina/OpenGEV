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
 * \brief The AbstractCommandHandler class is generic handler
 * for incoming command
 */
class AbstractStreamDataHandler : public AbstractPacketHandler
{
public:
    /**
     * @brief AbstractStreamDataHandler
     * @param target component where the new message should be applied
     * @param packetFormat packet format
     * @param datagram datagram received
     * @param senderAddress who send the datagram
     * @param senderPort destination port of the datagram
     */
    AbstractStreamDataHandler(GVComponent* target,
                           quint32 packetFormat,
                           QByteArray datagram,
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
    static quint32 readRequestPacketFormat(QByteArray* datagram);

    /**
     * @brief readRequestPacketId extract packetid from a datagram
     * @param datagram
     * @return packetid
     */
    static quint32 readRequestPacketId(QByteArray* datagram);

    /**
     * @brief readRequestBlockId extract block id from a datagram
     * @param datagram
     * @return block id
     */
    static quint64 readRequestBlockId(QByteArray* datagram);

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

    /**
     * @brief toString
     * @return handler major info as string
     */
    virtual std::string toString();

protected:

    quint32 requestPacketFormat;

    quint64 requestBlockId;

    quint32 requestPacketId;

    quint16 getAckHeaderLength() final;

    /**
     * @brief getAckHeader
     * @return header for ack message
     */
    virtual char* getAckHeader() final;

    /**
     * @brief checkHeader check received datagram
     * @return true if header is good
     */
    virtual bool checkHeader() final;
};

#endif // ABSTRACTSTREAMDATAHANDLER_H

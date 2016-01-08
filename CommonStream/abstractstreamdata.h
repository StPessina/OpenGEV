#ifndef STREAMDATALEADER_H
#define STREAMDATALEADER_H

#define HEADER_LENGTH 8

#include <QByteArray>

#include <string>
#include <QHostAddress>

#include <boost/detail/endian.hpp>

#include "CommonComponent/gvcomponent.h"

#include "CommonPacket/abstractpacket.h"
#include "CommonPacket//conversionutils.h"
#include "CommonPacket/status.h"

#include "CommonStream/packetformat.h"

/**
 * @brief The AbstractStreamData class provide generic rapresentation for stream data packet.
 * For each AbstractStreamData no ack message is required. So no target class will be specified
 * and execution is empty for each class that inherit AbstractStreamData.
 * In this implementation GigE vision extendDI is always enabled (GigE vision V2.0) so blockId is 64 bit
 * and packet id is 32 bit.
 */
class AbstractStreamData : public AbstractPacket
{
public:
    /**
     * @brief AbstractStreamData constructor
     * @param destAddress
     * @param destPort
     * @param packetFormat
     * @param blockId64
     * @param packetId32
     */
    AbstractStreamData(QHostAddress destAddress, quint16 destPort,
                       PacketFormat packetFormat, quint64 blockId64, quint32 packetId32);


    /**
     * @brief setFlagPacketResend method marks this packet as resend
     */
    virtual void setFlagPacketResend() final;

    /**
     * @brief ~AbstractCommand decostructor
     */
    virtual ~AbstractStreamData();

    /**
     * @brief executeAnswer no execution is required for this kind of messages
     * @param answer
     * @return 0 always
     */
    virtual int executeAnswer(const QByteArray &answer);

    /**
     * @brief toString
     * @return string value of the value
     */
    virtual std::string toString();

    /**
     * @brief getBlockId64 blockId of the stream data
     * @return blockId
     */
    virtual quint64 getBlockId64() final;

    /**
     * @brief getPacketId32
     * @return
     */
    virtual quint32 getPacketId32() final;

    /**
     * @brief getPacketFormat
     * @return packet format
     */
    virtual PacketFormat getPacketFormat() final;

protected:

    virtual quint16 getPacketHeaderLength() final;

    /*!
     * \brief getHeaderFlagFirstBits for custom bit flag redefine
     * Standard flag
     * \return flag bits
     */
    virtual short getHeaderFlag() final;

    /*!
     * \brief appendPacketHeader
     * \return char* with header
     */
    virtual void appendPacketHeader(QByteArray &datagram) final;

    /**
     * @brief packetId32 message packetId
     */
    quint32 packetId32;

private:

    /**
     * @brief status execution status (always SUCCESS)
     */
    Status status = GEV_STATUS_SUCCESS;

    /**
     * @brief flagResendError identify resend packet error
     */
    bool flagResendError = false; //CR-203st

    /**
     * @brief flagPreviousBlockDropped true if previous block is dropped
     */
    bool flagPreviousBlockDropped = false;

    /**
     * @brief flagPacketResend identify this packet as resent
     */
    bool flagPacketResend = false;

    /**
     * @brief extendDI always true
     */
    bool extendDI = true;

    /**
     * @brief packetFormat message type
     */
    PacketFormat packetFormat;

    /**
     * @brief blockId64 message block id
     */
    quint64 blockId64;
};

#endif // STREAMDATALEADER_H

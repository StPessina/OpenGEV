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
 * @brief The AbstractStreamData class provide generic rapresentation for stream data packet
 */
class AbstractStreamData : public AbstractPacket
{
public:
    /**
     * @brief AbstractStreamData constructor
     * @param target is the component where the command will be executed
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

    virtual int executeAnswer(const QByteArray &answer);

    /**
     * @brief toString
     * @return string value of the value
     */
    virtual std::string toString();

    /**
     * @brief getBlockId64
     * @return
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

    virtual quint16 getHeaderLength() final;

    /*!
     * \brief getHeaderFlagFirstBits for custom bit flag redefine
     * Standard flag
     * \return flag bits
     */
    virtual short getHeaderFlag() final;

    /*!
     * \brief getHeader
     * \return char* with header
     */
    virtual void appendHeader(QByteArray &datagram) final;

    quint32 packetId32;

private:

    Status status = GEV_STATUS_SUCCESS;

    bool flagResendError = false; //CR-203st

    bool flagPreviousBlockDropped = false;

    bool flagPacketResend = false;

    bool extendDI = true;

    PacketFormat packetFormat;

    quint64 blockId64;
};

#endif // STREAMDATALEADER_H

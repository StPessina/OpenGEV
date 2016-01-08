#ifndef ABSTRACTPACKET_H
#define ABSTRACTPACKET_H

#include <QByteArray>

#include <string>
#include <QHostAddress>

#include <boost/detail/endian.hpp>

#include "CommonComponent/gvcomponent.h"

#include "CommonPacket/conversionutils.h"

/**
 * @brief The AbstractPacket class provide generic rapresentation for a packet. It can be
 * inherited to create packets for GigE commands or stream packets and so on. UDPChannel can accept this
 * kind children of this abstract class for messages sending.
 */
class AbstractPacket
{
public:
    /**
     * @brief AbstractPacket constructor
     * @param target is the component where the command will be executed when ack is received
     * @param destAddress
     * @param destPort
     * @param commandCode
     * @param ackCommandCode aspected ack command code
     * @param reqId
     * @param requireAck if this message require ack
     * @param broadcast if this message is a broadcast message
     */
    AbstractPacket(GVComponent* const target, QHostAddress destAddress, quint16 destPort,
                   quint16 reqId, bool requireAck, bool broadcast);

    /**
     * @brief ~AbstractCommand decostructor
     */
    virtual ~AbstractPacket();

    /**
     * @brief getPacketDatagram method
     * @return the datagram of the packet
     */
    virtual const QByteArray& getPacketDatagram() final;

    /**
     * @brief getDestinationAddress method
     * @return the ip address of the destination of the command
     */
    virtual QHostAddress getDestinationAddress() final;

    /**
     * @brief getDestionationPort method
     * @return the port of the destination of the command
     */
    virtual quint16 getDestionationPort() final;

    /**
     * @brief setRequestId method
     * @param reqId new request id value
     */
    virtual void setRequestId(quint16 reqId) final;

    /**
     * @brief getRequestId method
     * @return the request id
     */
    virtual quint16 getRequestId() final;

    /**
     * @brief isAckRequired method
     * @return true if the ack is required
     */
    virtual bool isAckRequired() final;

    /**
     * @brief isBroadcastMessage method
     * @return true if the message is broadcast
     */
    virtual bool isBroadcastMessage() final;

    /**
     * @brief setAnswer
     * @param answer
     */
    virtual void setAnswer(const QByteArray &answer) final;

    /**
     * @brief executeAnswer method
     * @param answer received for the command
     * @return 0 if the answer is processed and executed successfully
     */
    virtual int executeAnswer(const QByteArray &answer) = 0;

    /**
     * @brief toString
     * @return string value of the value
     */
    virtual std::string toString();

protected:

    /**
     * @brief target where the packet will be executed
     */
    GVComponent* target;

    /**
     * @brief getPacketBodyLength
     * @return length of the command request
     */
    virtual quint16 getPacketHeaderLength() = 0;

    /*!
     * \brief getHeader
     * \return QByteArray with header
     */
    virtual void appendPacketHeader(QByteArray &datagram) = 0;

    /**
     * @brief getPacketBodyLength
     * @return length of the command request
     */
    virtual quint16 getPacketBodyLength() = 0;

    /**
     * @brief appendPacketBody
     * @param datagram where specific datagram should be appended
     */
    virtual void appendPacketBody(QByteArray &datagram) = 0;

    /**
     * @brief haveAnswer method
     * @return if the answer was set
     */
    virtual bool haveAnswer() final;

    /**
     * @brief getAnswer method
     * @return the answer received for this command, null if no answer is received
     */
    virtual const QByteArray* getAnswer() final;

    /**
     * @brief answer received for this command
     */
    const QByteArray *answer;



private:

    /**
     * @brief fixed reference to datagram
     */
    QByteArray datagram;

    /**
     * @brief destAddress the destination address
     */
    QHostAddress destAddress;

    /**
     * @brief destPort the destination port
     */
    quint16 destPort;

    /**
     * @brief reqId the request id
     */
    quint16 reqId;

    /**
     * @brief requireACK
     */
    bool requireACK;

    /**
     * @brief broadcast, true if this command is a broadcast message
     */
    bool broadcast;
};

#endif // ABSTRACTPACKET_H

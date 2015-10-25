#ifndef ABSTRACTCOMMAND_H
#define ABSTRACTCOMMAND_H

#define HEADER_LENGTH 8

#include <QByteArray>

#include <string>
#include <QHostAddress>

#include <boost/detail/endian.hpp>

#include "CommonComponent/gvcomponent.h"

/**
 * @brief The AbstractCommand class provide generic rapresentation for a command
 */
class AbstractCommand
{
public:
    /**
     * @brief AbstractCommand constructor
     * @param target is the component where the command will be executed
     * @param destAddress
     * @param destPort
     * @param commandCode
     * @param ackCommandCode aspected ack command code
     * @param reqId
     * @param requireAck if this message require ack
     * @param broadcast if this message is a broadcast message
     */
    AbstractCommand(GVComponent* target, QHostAddress destAddress, quint16 destPort,
                    int commandCode, int ackCommandCode, int reqId, bool requireAck, bool broadcast);

    /**
     * @brief ~AbstractCommand decostructor
     */
    virtual ~AbstractCommand();

    /**
     * @brief getCommandDatagram method
     * @return the datagram of the command
     */
    virtual QByteArray* getCommandDatagram() final;

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
     * @brief getCommandCode method
     * @return command code
     */
    virtual int getCommandCode() final;

    /**
     * @brief setRequestId method
     * @param reqId new request id value
     */
    virtual void setRequestId(int reqId) final;

    /**
     * @brief getRequestId method
     * @return the request id
     */
    virtual int getRequestId() final;

    /**
     * @brief isAckRequired method
     * @return true if the ack is required
     */
    virtual bool isAckRequired() final;

    /**
     * @brief getLengthWithoutHeader
     * @return length of the command request
     */
    virtual int getLengthWithoutHeader() = 0;

    /**
     * @brief isBroadcastMessage method
     * @return true if the message is broadcast
     */
    virtual bool isBroadcastMessage() final;

    /**
     * @brief checkAckHeader method
     * @param answer
     * @return true if the header of the message is a valid answer
     */
    virtual bool checkAckHeader(QByteArray answer) final;

    /**
     * @brief executeAnswer method
     * @param answer received for the command
     * @return 0 if the answer is processed and executed successfully
     */
    virtual int executeAnswer(QByteArray answer) = 0;

    /**
     * @brief toString
     * @return string value of the value
     */
    std::string toString();

protected:

    /**
     * @brief target where the command will be executed
     */
    GVComponent* target;

    /*!
     * \brief getHeaderFlagFirstBits for custom bit flag redefine
     * Standard flag
     * bit 0: free for command specific
     * bit 1: free for command specific
     * bit 2: free for command specific
     * bit 3: free for command specific
     * bit 4: reserved set it to 0
     * bit 5: reserved set it to 0
     * bit 6: reserved set it to 0
     * bit 7: ack required
     * \return flag bits
     */
    short getHeaderFlag();

    /**
     * @brief getCommandDatagramWithoutHeader
     * @return the command datagram
     */
    virtual char* getCommandDatagramWithoutHeader() = 0;

    /**
     * @brief haveAnswer method
     * @return if the answer was set
     */
    virtual bool haveAnswer() final;

    /**
     * @brief getAnswer method
     * @return the answer received for this command, null if no answer is received
     */
    virtual QByteArray getAnswer() final;

protected:
    /**
     * @brief answer received for this command
     */
    QByteArray answer;

private:

    /*!
     * \brief getHeader
     * standard GV command header
     * byte 0: 0x42
     * byte 1: control flag (bit 7 = requireACK
     * byte 2: command code MSB
     * byte 3: command code LSB
     * byte 4: length without header MSB
     * byte 5: length without header LSB
     * byte 6: req_id MSB
     * byte 7: req_id LSB
     * \return char* with header
     */
    virtual char* getHeader() final;

    /**
     * @brief destAddress the destination address
     */
    QHostAddress destAddress;

    /**
     * @brief destPort the destination port
     */
    quint16 destPort;

    /**
     * @brief commandCode the command code
     */
    int commandCode;

    /**
     * @brief ackCommandCode the aspected ack code
     */
    int ackCommandCode;

    /**
     * @brief reqId the request id
     */
    int reqId;

    /**
     * @brief requireACK
     */
    bool requireACK;

    /**
     * @brief broadcast, true if this command is a broadcast message
     */
    bool broadcast;
};

#endif // ABSTRACTCOMMAND_H

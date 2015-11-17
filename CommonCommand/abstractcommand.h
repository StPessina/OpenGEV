#ifndef ABSTRACTCOMMAND_H
#define ABSTRACTCOMMAND_H

#define HEADER_LENGTH 8

#include <QByteArray>

#include <string>
#include <QHostAddress>

#include <boost/detail/endian.hpp>

#include "CommonComponent/gvcomponent.h"

#include "CommonPacket/abstractpacket.h"
#include "CommonPacket//conversionutils.h"

/**
 * @brief The AbstractCommand class provide generic rapresentation for a command
 */
class AbstractCommand : public AbstractPacket
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
                    quint16 commandCode, quint16 ackCommandCode, quint16 reqId, bool requireAck, bool broadcast);

    /**
     * @brief ~AbstractCommand decostructor
     */
    virtual ~AbstractCommand();

    /**
     * @brief getCommandCode method
     * @return command code
     */
    virtual quint16 getCommandCode() final;

    /**
     * @brief getLengthWithoutHeader
     * @return length of the command request
     */
    virtual quint16 getLengthWithoutHeader() = 0;

    /**
     * @brief checkAckHeader method
     * @param answer
     * @return true if the header of the message is a valid answer
     */
    virtual bool checkAckHeader(QByteArray answer) final;

    /**
     * @brief getStatusCode
     * @param answer received
     * @return status code
     */
    virtual short getStatusCodeFromAnswer(QByteArray answer) final;

    /**
     * @brief getStatusCode
     * @return status code
     */
    virtual short getStatusCode() final;

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

    quint16 getHeaderLength();

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
     * @brief getCommandDatagramWithoutHeader
     * @return the command datagram
     */
    virtual char* getCommandDatagramWithoutHeader() = 0;

protected:
    /**
     * @brief answer received for this command
     */
    QByteArray answer;

private:

    /**
     * @brief commandCode the command code
     */
    quint16 commandCode;

    /**
     * @brief ackCommandCode the aspected ack code
     */
    quint16 ackCommandCode;
};

#endif // ABSTRACTCOMMAND_H

#ifndef ABSTRACTCOMMAND_H
#define ABSTRACTCOMMAND_H

#define HEADER_LENGTH 8

#include <QByteArray>

#include <string>
#include <QHostAddress>

#include <boost/detail/endian.hpp>

#include "gvcomponent.h"

class AbstractCommand
{
public:
    AbstractCommand(GVComponent* target, QHostAddress destAddress, quint16 destPort,
                    int commandCode, int ackCommandCode, int reqId, bool requireAck, bool broadcast);

    virtual ~AbstractCommand();

    virtual QByteArray* getCommandDatagram() final;

    virtual QHostAddress getDestinationAddress() final;

    virtual quint16 getDestionationPort() final;

    virtual int getCommandCode() final;

    virtual void setRequestId(int reqId) final;

    virtual int getRequestId() final;

    virtual bool isAckRequired() final;

    virtual int getLengthWithoutHeader() = 0;

    virtual bool isBroadcastMessage() final;

    virtual bool checkAckHeader(QByteArray answer) final;

    virtual int executeAnswer(QByteArray answer) = 0;

    std::string toString();

protected:

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

    virtual char* getCommandDatagramWithoutHeader() = 0;

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

    QHostAddress destAddress;
    quint16 destPort;
    int commandCode;
    int ackCommandCode;
    int reqId;
    bool requireACK;
    bool broadcast;
};

#endif // ABSTRACTCOMMAND_H

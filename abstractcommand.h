#ifndef ABSTRACTCOMMAND_H
#define ABSTRACTCOMMAND_H

#include <QByteArray>

class AbstractCommand
{
public:
    AbstractCommand(int commandCode, int reqId, bool requireAck);

    virtual QByteArray* getCommandDatagram() final;

    virtual int getCommandCode() final;

    virtual int getRequestId() final;

    virtual bool isAckRequired() final;

    virtual int getLength();

protected:
    /*!
     * \brief getHeaderFlagFirstBits for custom bit flag redefine
     * \return flag bits
     */
    char* getHeaderFlag();

    virtual QByteArray* getCommandDatagramWithoutHeader();

private:

    virtual char* getHeader() final;

    int commandCode;
    int reqId;
    bool requireACK;
};

#endif // ABSTRACTCOMMAND_H

#ifndef PACKETRESENDCOMMAND_H
#define PACKETRESENDCOMMAND_H

#include "CommonCommand/abstractcommand.h"

#include "ApplicationCommand/applicationcommandcode.h"

#include "DeviceCommandHandler/deviceackcode.h"
/**
 * @brief The ReadRegisterCommand class provide cmd for register read
 */
class PacketResendCommand : public AbstractCommand
{
public:

    /**
     * @brief ReadRegisterCommand constructor (for single read)
     * @param target is the application where discovered devices will be inserted
     * @param registerAddress to read
     * @param destinationAddress
     * @param destinationPort
     */
    PacketResendCommand(GVComponent* target,
                        QHostAddress destinationAddress,
                        quint16 destinationPort,
                        quint16 streamChannelNr,
                        quint64 blockId,
                        quint32 firstPacketId,
                        quint32 lastPacketId);

    /**
     * @brief ~ReadRegisterCommand deconstructor
     */
    virtual ~PacketResendCommand();

    /**
     * @brief getLengthWithoutHeader method
     * @return the length of discorery command
     */
    quint16 getLengthWithoutHeader();

    /**
     * @brief getCommandDatagramWithoutHeader method
     * @return datagram
     */
    void appendPacketDatagramWithoutHeader(QByteArray &datagram);

    /**
     * @brief executeAnswer method will execute the command on the target
     * @param answer datagram received from a devices
     * @return 0 if the command is successfully executed
     */
    int executeAnswer(const QByteArray &answer);

    virtual short getHeaderFlag();

private:
    quint32 streamChannelNr;
    quint64 blockId;
    quint32 firstPacketId;
    quint32 lastPacketId;
};

#endif // PACKETRESENDCOMMAND_H

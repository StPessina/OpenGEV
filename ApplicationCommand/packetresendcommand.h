#ifndef PACKETRESENDCOMMAND_H
#define PACKETRESENDCOMMAND_H

#include "CommonCommand/abstractcommand.h"

#include "ApplicationCommand/applicationcommandcode.h"

#include "DeviceCommandHandler/deviceackcode.h"
/**
 * @brief The PacketResendCommand class provide cmd to request packet
 * resend on a stream channel. Normally no ack will be received from the device
 * for this command.
 */
class PacketResendCommand : public AbstractCommand
{
public:

    /**
     * @brief PacketResendCommand constructor
     * @param target stream channel that request stream data resend
     * @param destinationAddress
     * @param destinationPort
     * @param streamChannelNr stream channel number
     * @param blockId
     * @param firstPacketId first packet id to resend
     * @param lastPacketId last packet id to resend
     */
    PacketResendCommand(GVComponent* target,
                        QHostAddress destinationAddress,
                        quint16 destinationPort,
                        quint16 streamChannelNr,
                        quint64 blockId,
                        quint32 firstPacketId,
                        quint32 lastPacketId);

    /**
     * @brief ~PacketResendCommand deconstructor
     */
    virtual ~PacketResendCommand();

    /**
     * @brief getPacketBodyLength method
     * @return the length of discorery command
     */
    quint16 getPacketBodyLength();

    /**
     * @brief appendPacketBody method
     * @param datagram where append data
     */
    void appendPacketBody(QByteArray &datagram);

    /**
     * @brief executeAnswer method will execute the command on the target
     * @param answer datagram received from a devices
     * @return 0 if the command is successfully executed
     */
    int executeAnswer(const QByteArray &answer);

    virtual short getHeaderFlag();

private:
    /**
     * @brief streamChannelNr stream channel that request retrasmission
     */
    quint32 streamChannelNr;

    /**
     * @brief blockId requested
     */
    quint64 blockId;

    /**
     * @brief firstPacketId requested
     */
    quint32 firstPacketId;

    /**
     * @brief lastPacketId requested
     */
    quint32 lastPacketId;
};

#endif // PACKETRESENDCOMMAND_H

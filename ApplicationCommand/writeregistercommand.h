#ifndef WRITEREGISTERCOMMAND_H
#define WRITEREGISTERCOMMAND_H

#include <vector>
#include <unordered_map>

#include "CommonCommand/abstractcommand.h"

#include "ApplicationCommand/applicationcommandcode.h"

#include "DeviceCommandHandler/deviceackcode.h"

/**
 * @brief The WriteRegisterCommand class implements write register command
 * R-166cd
 */
class WriteRegisterCommand : public AbstractCommand
{
public:
    /**
     * @brief WriteRegisterCommand constructor (for single write)
     * @param target
     * @param registerAddress to write
     * @param value to write
     * @param destinationAddress
     * @param destinationPort
     */
    WriteRegisterCommand(GVComponent* target,
                        int registerAddress,
                        int value,
                        QHostAddress destinationAddress,
                        quint16 destinationPort);

    /**
     * @brief WriteRegisterCommand constructor (for multiple write register)
     * @param target
     * @param registerAddresses to write
     * @param registerValues to write
     * @param destinationAddress
     * @param destinationPort
     */
    WriteRegisterCommand(GVComponent* target,
                        std::vector<int> registerAddresses,
                         std::vector<int> registerValues,
                        QHostAddress destinationAddress,
                        quint16 destinationPort);

    /**
     * @brief ~WriteRegisterCommand deconstructor
     */
    virtual ~WriteRegisterCommand();

    /**
     * @brief getPacketBodyLength method
     * @return the length of discorery command
     */
    quint16 getPacketBodyLength();

    /**
     * @brief appendPacketBody method
     * @return datagram where append data
     */
    void appendPacketBody(QByteArray &datagram);

    /**
     * @brief executeAnswer method will execute the command on the target
     * @param answer datagram received from a devices
     * @return 0 if the command is successfully executed
     */
    int executeAnswer(const QByteArray &answer);

private:
    /**
     * @brief registersData store registers addresses and values to write
     */
    std::unordered_map<int, int> registersData;
};

#endif // WRITEREGISTERCOMMAND_H

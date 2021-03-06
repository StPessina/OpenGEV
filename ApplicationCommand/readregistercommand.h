#ifndef READREGISTERCOMMAND_H
#define READREGISTERCOMMAND_H

#include <vector>
#include <unordered_map>

#include "CommonCommand/abstractcommand.h"

#include "ApplicationCommand/applicationcommandcode.h"

#include "DeviceCommandHandler/deviceackcode.h"
/**
 * @brief The ReadRegisterCommand class provide cmd for register/s read
 */
class ReadRegisterCommand : public AbstractCommand
{
public:

    /**
     * @brief ReadRegisterCommand constructor (for single read)
     * @param target
     * @param registerAddress to read
     * @param destinationAddress
     * @param destinationPort
     */
    ReadRegisterCommand(GVComponent* target,
                        int registerAddress,
                        QHostAddress destinationAddress,
                        quint16 destinationPort);

    /**
     * @brief ReadRegisterCommand constructor (for multiple read register). Only if device
     * support multiple read (Check capability register on the device)
     * @param target is the application where discovered devices will be inserted
     * @param registerAddresses to read
     * @param destinationAddress
     * @param destinationPort
     */
    ReadRegisterCommand(GVComponent* target,
                        std::vector<int> registerAddresses,
                        QHostAddress destinationAddress,
                        quint16 destinationPort);

    /**
     * @brief ~ReadRegisterCommand deconstructor
     */
    virtual ~ReadRegisterCommand();

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

    /**
     * @brief getRegisterValue for read register value after request (single read)
     * @return value of the requested register
     */
    int getRegisterValue();

    /**
     * @brief getRegisterValue for read register value after request (multiple read)
     * @param registerAddress
     * @return value of register if address is valid
     */
    int getRegisterValue(int registerAddress);

private:
    /**
     * @brief registersData hash map to store values readed
     */
    std::unordered_map<int, int> registersData;
};

#endif // READREGISTERCOMMAND_H

#ifndef READREGISTERCOMMAND_H
#define READREGISTERCOMMAND_H

#include <vector>
#include <unordered_map>

#include "CommonCommand/abstractcommand.h"

#include "ApplicationCommand/applicationcommandcode.h"

#include "DeviceCommandHandler/deviceackcode.h"
/**
 * @brief The ReadRegisterCommand class provide cmd for register read
 */
class ReadRegisterCommand : public AbstractCommand
{
public:

    /**
     * @brief ReadRegisterCommand constructor (for single read)
     * @param target is the application where discovered devices will be inserted
     * @param registerAddress to read
     * @param destinationAddress
     * @param destinationPort
     */
    ReadRegisterCommand(GVComponent* target,
                        int registerAddress,
                        QHostAddress destinationAddress,
                        quint16 destinationPort);

    /**
     * @brief ReadRegisterCommand constructor (for multiple read register)
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
     * @brief getLengthWithoutHeader method
     * @return the length of discorery command
     */
    quint16 getLengthWithoutHeader();

    /**
     * @brief getCommandDatagramWithoutHeader method
     * @return datagram
     */
    char* getPacketDatagramWithoutHeader();

    /**
     * @brief executeAnswer method will execute the command on the target
     * @param answer datagram received from a devices
     * @return 0 if the command is successfully executed
     */
    int executeAnswer(QByteArray answer);

    /**
     * @brief getRegisterValue
     * @return value of the requested register
     */
    int getRegisterValue();

    /**
     * @brief getRegisterValue
     * @param registerAddress
     * @return
     */
    int getRegisterValue(int registerAddress);

private:
    std::unordered_map<int, int> registersData;
};

#endif // READREGISTERCOMMAND_H

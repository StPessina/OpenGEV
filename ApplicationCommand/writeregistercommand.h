#ifndef WRITEREGISTERCOMMAND_H
#define WRITEREGISTERCOMMAND_H

#include <vector>
#include <unordered_map>

#include "CommonMessages/abstractcommand.h"

#include "ApplicationCommand/applicationcommandcode.h"

#include "DeviceMessageHandler/deviceackcode.h"

#include "CommonMessages/conversionutils.h"

/**
 * @brief The WriteRegisterCommand class implements read register command
 * R-166cd
 */
class WriteRegisterCommand : public AbstractCommand
{
public:
    /**
     * @brief WriteRegisterCommand constructor (for single read)
     * @param target is the application where discovered devices will be inserted
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
     * @brief WriteRegisterCommand constructor (for multiple read register)
     * @param target is the application where discovered devices will be inserted
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
     * @brief getLengthWithoutHeader method
     * @return the length of discorery command
     */
    quint16 getLengthWithoutHeader();

    /**
     * @brief getCommandDatagramWithoutHeader method
     * @return datagram
     */
    char* getCommandDatagramWithoutHeader();

    /**
     * @brief executeAnswer method will execute the command on the target
     * @param answer datagram received from a devices
     * @return 0 if the command is successfully executed
     */
    int executeAnswer(QByteArray answer);
};

#endif // WRITEREGISTERCOMMAND_H

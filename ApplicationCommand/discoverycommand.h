#ifndef DISCOVERYCOMMAND_H
#define DISCOVERYCOMMAND_H

#include "opengv_global.h"

#include "CommonMessages/abstractcommand.h"

#include "Application/gvapplication.h"

#include "ApplicationCommand/applicationcommandcode.h"

#include "DeviceMessageHandler/deviceackcode.h"

#include "CommonMessages/conversionutils.h"

using namespace std;

/**
 * @brief The DiscoveryCommand class provide datagram for device discover
 */
class DiscoveryCommand : public AbstractCommand
{
public:
    /**
     * @brief DiscoveryCommand constructor (for broadcast command)
     * @param target is the application where discovered devices will be inserted
     */
    DiscoveryCommand(GVComponent* target);

    /**
     * @brief ~DiscoveryCommand deconstructor
     */
    virtual ~DiscoveryCommand();

    /**
     * @brief DiscoveryCommand constructor (with unicast specification)
     * @param target is the application where discovered devices will be inserted
     * @param destinationAddress
     * @param destinationPort
     */
    DiscoveryCommand(GVComponent* target, QHostAddress destinationAddress, quint16 destinationPort);

    /**
     * @brief getLengthWithoutHeader method
     * @return the length of discorery command
     */
    int getLengthWithoutHeader();

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

#endif // DISCOVERYCOMMAND_H

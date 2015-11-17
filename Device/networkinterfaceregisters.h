#ifndef NETWORKINTERFACEREGISTERS_H
#define NETWORKINTERFACEREGISTERS_H

#include <QtNetwork/QNetworkInterface>
#include <QNetworkAddressEntry>

#include <map>
#include <QString>

#include "Device/deviceregisters.h"

#include "CommonBootstrapRegister/bootstrapregister.h"

using namespace std;

/**
 * @brief The NetworkInterfaceRegisters class store bootstrap register for a device network interface
 */
class NetworkInterfaceRegisters
{
public:
    /**
     * @brief NetworkInterfaceRegisters
     * @param netInterface
     * @param interfaceNumber
     */
    NetworkInterfaceRegisters(QNetworkInterface netInterface, int interfaceNumber);

    /**
     * @brief ~NetworkInterfaceRegisters deconstructor
     */
    virtual ~NetworkInterfaceRegisters();

    /**
     * @brief getRegister
     * @param offsetRegisterCode (use defined value)
     * @return network register if exist
     */
    BootstrapRegister *getRegister(int offsetRegisterCode);

    /**
     * @brief getRegisterByAbsoluteRegCode
     * @param regCode
     * @return network register if exist
     */
    BootstrapRegister *getRegisterByAbsoluteRegCode(int regCode);

    /**
     * @brief getInterfaceNumber
     * @return interface number
     */
    int getInterfaceNumber();

private:

    int interfaceNumber;

    /**
     * @brief map of network register
     */
    map<int,BootstrapRegister*> registers;

    /**
     * @brief initRegisterMap
     */
    void initRegisterMap();

    void updateNetworkStatus();

    QNetworkInterface netInterface;

    int deviceMACAddressHigh;
    int deviceMACAddessLow;
    int networkInterfaceCapability;
    int networkInterfaceConfiguration;
    int currentIPAddress;
    int currentSubnetMask;
    int currentDefaultGateway;
    int persintentIPAddress;
    int persistentSubnetMask;
    int persistentDefaultGateway;
    int linkSpeed;

    /**
     * @brief getInterfaceListWithoutLoopBack
     * @param interfaceNumber
     * @return network interface from interface number
     */
    QNetworkInterface getInterfaceListWithoutLoopBack(int interfaceNumber);
};

#endif // NETWORKINTERFACEREGISTERS_H

#ifndef NETWORKINTERFACEREGISTERS_H
#define NETWORKINTERFACEREGISTERS_H

#include <QtNetwork/QNetworkInterface>
#include <QNetworkAddressEntry>

#include <map>
#include <QString>
#include "deviceregisters.h"

#include "bootstrapregister.h"

using namespace std;

class NetworkInterfaceRegisters
{
public:
    NetworkInterfaceRegisters(int interfaceNumber);

    BootstrapRegister *getRegister(int offsetRegisterCode);

    int getInterfaceNumber();

private:

    int interfaceNumber;

    map<int,BootstrapRegister*> registers;

    void initRegisterMap();

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

    QNetworkInterface getInterfaceListWithoutLoopBack(int interfaceNumber);
};

#endif // NETWORKINTERFACEREGISTERS_H

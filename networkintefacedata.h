#ifndef NETWORKINTEFACEDATA_H
#define NETWORKINTEFACEDATA_H

class NetworkIntefaceData
{
public:
    NetworkIntefaceData(int interfaceNumber);

    void loadPersistenceDataFromFile();

    void savePersistenceDataToFile();

    void updateDataFromInterface();

    int getInterfaceNumber();

private:
    int interfaceNumber;

    int MACAddressHigh;
    int MACAddressLow;
    int networkInterfaceCapability;
    int networkInterfaceConfiguration;

    int currentIPAddress;
    int currentSubnetMask;
    int currentDefaultGateway;

    int persistentIPAddress;
    int persistentSubnetMask;
    int persistentGatewayMask;

    int linkSpeed;
};

#endif // NETWORKINTEFACEDATA_H

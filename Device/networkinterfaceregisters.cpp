#include "networkinterfaceregisters.h"

NetworkInterfaceRegisters::NetworkInterfaceRegisters(QNetworkInterface netInterface, int interfaceNumber)
{
    this->netInterface = netInterface;
    this->interfaceNumber = interfaceNumber;
    initRegisterMap();
    updateNetworkStatus();

    BootstrapRegister* networkCapabilitieReg = (BootstrapRegister*) registers.at(
                    DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_NETWORK_INTERFACE_CAPABILITIES));
    networkCapabilitieReg->setBit(29); //Link-local address
    networkCapabilitieReg->setBit(30); //DHCP
    //networkCapabilitieReg->setBit(31); //IP persistent address
}

NetworkInterfaceRegisters::~NetworkInterfaceRegisters()
{
    foreach(auto reg, registers)
        delete reg.second;
}

BootstrapRegister *NetworkInterfaceRegisters::getRegister(int offsetRegisterCode)
{
    BootstrapRegister* reg = (BootstrapRegister*) registers.at(
                DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, offsetRegisterCode));
    return reg;
}

BootstrapRegister *NetworkInterfaceRegisters::getRegisterByAbsoluteRegCode(int regCode)
{
    int regType = DeviceRegisterConverter::getInterfaceNumberFromNetworkRegister(regCode);
    return getRegister(regType);
}

int NetworkInterfaceRegisters::getInterfaceNumber()
{
    return interfaceNumber;
}

void NetworkInterfaceRegisters::initRegisterMap()
{
    deviceMACAddressHigh=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_DEVICE_MAC_ADD_HIGH);
    deviceMACAddessLow=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_DEVICE_MAC_ADD_LOW);
    networkInterfaceCapability=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_NETWORK_INTERFACE_CAPABILITIES);
    networkInterfaceConfiguration=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_NETWORK_INTERFACE_CONF);
    currentIPAddress=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_CURRENT_IP_ADD);
    currentSubnetMask=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_CURRENT_SUBNET_MASK);
    currentDefaultGateway=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_CURRENT_DEFAULT_GATEWAY);
    persintentIPAddress=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_PERSISTENT_IP_ADD);
    persistentSubnetMask=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_PERSISTENT_SUBNET_MASK);
    persistentDefaultGateway=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_PERSISTENT_DEFAULT_GATEWAY);
    linkSpeed=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_LINK_SPEED);

    registers[deviceMACAddressHigh]
            = new  BootstrapRegister(deviceMACAddressHigh, "Device MAC Address - High (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ, 4);
    registers[deviceMACAddessLow]
     = new  BootstrapRegister(deviceMACAddessLow, "Device MAC Address - Low (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ, 4);
    registers[networkInterfaceCapability]
     = new  BootstrapRegister(networkInterfaceCapability, "Network Interface Capability (Network interface #" +
                                     to_string(interfaceNumber) + ")", RA_READ, 4);
    registers[networkInterfaceConfiguration]
     = new  BootstrapRegister(networkInterfaceConfiguration, "Network Interface Configuration (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ_WRITE, 4);
    registers[currentIPAddress]
     = new  BootstrapRegister(currentIPAddress, "Current IP Address (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ, 4);
    registers[currentSubnetMask]
     = new  BootstrapRegister(currentSubnetMask, "Current Subnet Mask (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ, 4);
    registers[currentDefaultGateway]
     = new  BootstrapRegister(currentDefaultGateway, "Current Default Gateway (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ, 4);
    registers[persintentIPAddress]
     = new  BootstrapRegister(persintentIPAddress, "Persistent IP Address (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ_WRITE, 4);
    registers[persistentSubnetMask]
     = new  BootstrapRegister(persistentSubnetMask, "Persistent Subnet Mask (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ_WRITE, 4);
    registers[persistentDefaultGateway]
     = new  BootstrapRegister(persistentDefaultGateway, "Persistent Default Gateway (Network interface #" +
                                     to_string(interfaceNumber) + ")", RA_READ_WRITE, 4);
    registers[linkSpeed]
     = new  BootstrapRegister(linkSpeed, "Link Speed (Network interface #" +
                              to_string(interfaceNumber) + ")",RA_READ, 4);
}

void NetworkInterfaceRegisters::updateNetworkStatus()
{
    QString MACAddr = netInterface.hardwareAddress();
    bool* ok = new bool;
    int MACHigh1 = MACAddr.section(':',1,1).toInt(ok,16);
    int MACHigh2 = MACAddr.section(':',0,0).toInt(ok,16);
    ((BootstrapRegister*) registers.at(
                DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber,
                                                                     REG_DEVICE_MAC_ADD_HIGH)))
            ->setValueNumb((MACHigh1 | (MACHigh2 << 8))); //R-431cd

    int MACLow1 = MACAddr.section(':',5,5).toInt(ok,16);
    int MACLow2 = MACAddr.section(':',4,4).toInt(ok,16);
    int MACLow3 = MACAddr.section(':',3,3).toInt(ok,16);
    int MACLow4 = MACAddr.section(':',2,2).toInt(ok,16);
    ((BootstrapRegister*) registers.at(
                DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber,
                                                                     REG_DEVICE_MAC_ADD_LOW)))
            ->setValueNumb(MACLow1 | (MACLow2 << 8) | (MACLow3 << 16) | (MACLow4 << 24));
    delete ok;

    int ip = 0;
    if(netInterface.addressEntries().size()>0) {
        QNetworkAddressEntry host = netInterface.addressEntries().at(0);
        ip = host.ip().toIPv4Address();
    }
    ((BootstrapRegister*) registers.at(
                DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber,
                                                                     REG_CURRENT_IP_ADD)))
            ->setValueNumb(ip);

    int subnetMask = 0;
    if(netInterface.addressEntries().size()>0)
        subnetMask = (int) netInterface.addressEntries().at(0).netmask().toIPv4Address();
    ((BootstrapRegister*) registers.at(
                DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber,
                                                                     REG_CURRENT_SUBNET_MASK)))
            ->setValueNumb(subnetMask);

    ((BootstrapRegister*) registers.at(
                DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber,
                                                                     REG_CURRENT_DEFAULT_GATEWAY)))
            ->setValueNumb(0);

    //Persistent IP CR-447cd
    ((BootstrapRegister*) registers.at(
                DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber,
                                                                     REG_PERSISTENT_IP_ADD)))
            ->setValueNumb(0);

    //Persistent subnet mask CR-448cd
    ((BootstrapRegister*) registers.at(
                DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber,
                                                                     REG_PERSISTENT_SUBNET_MASK)))
            ->setValueNumb(0);

    //Persistent default gateway CR-449cd
    ((BootstrapRegister*) registers.at(
                DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber,
                                                                     REG_PERSISTENT_DEFAULT_GATEWAY)))
            ->setValueNumb(0);

    //Link speed CR-450cd
    ((BootstrapRegister*) registers.at(
                DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber,
                                                                     REG_LINK_SPEED)))
            ->setValueNumb(0);
}

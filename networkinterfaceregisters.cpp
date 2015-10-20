#include "networkinterfaceregisters.h"

NetworkInterfaceRegisters::NetworkInterfaceRegisters(int interfaceNumber)
{
    this->interfaceNumber = interfaceNumber;
    initRegisterMap();

    netInterface = QNetworkInterface::interfaceFromIndex(interfaceNumber);
}

BootstrapRegister *NetworkInterfaceRegisters::getRegister(int offsetRegisterCode)
{
    BootstrapRegister* reg = (BootstrapRegister*) registers.at(
                DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, offsetRegisterCode));

    if(offsetRegisterCode==deviceMACAddressHigh) {
        QString MACAddr = netInterface.hardwareAddress();
        int MACHigh = MACAddr.at(4).digitValue() | (MACAddr.at(5).digitValue() >> 8)
                | (MACAddr.at(6).digitValue() >> 16) | (MACAddr.at(7).digitValue() >> 24);
        reg->setValueNumb(MACHigh);
    }

    if(offsetRegisterCode==deviceMACAddessLow) {
        QString MACAddr = netInterface.hardwareAddress();
        int MACLow = MACAddr.at(0).digitValue() | (MACAddr.at(1).digitValue() >> 8)
                | (MACAddr.at(2).digitValue() >> 16) | (MACAddr.at(3).digitValue() >> 24);
        reg->setValueNumb(MACLow);
    }

    if(offsetRegisterCode==networkInterfaceCapability) {

    }

    if(offsetRegisterCode==networkInterfaceConfiguration) {

    }

    if(offsetRegisterCode==currentIPAddress) {
        //netInterface.
    }

    if(offsetRegisterCode==currentSubnetMask) {

    }

    if(offsetRegisterCode==currentDefaultGateway) {

    }

    if(offsetRegisterCode==persintentIPAddress) {

    }

    if(offsetRegisterCode==persistentSubnetMask) {

    }

    if(offsetRegisterCode==persistentDefaultGateway) {

    }

    if(offsetRegisterCode==linkSpeed) {

    }

    return reg;
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

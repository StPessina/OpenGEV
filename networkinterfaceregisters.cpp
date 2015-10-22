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

    switch (offsetRegisterCode) {
    case REG_DEVICE_MAC_ADD_HIGH:
    {
        QString MACAddrH = netInterface.hardwareAddress();
        int MACHigh = MACAddrH.at(0).digitValue() | (MACAddrH.at(1).digitValue() >> 8);
        reg->setValueNumb(MACHigh);
        break;
    }
    case REG_DEVICE_MAC_ADD_LOW:
    {
        QString MACAddrL = netInterface.hardwareAddress();
        int MACLow = MACAddrL.at(0).digitValue() | (MACAddrL.at(1).digitValue() >> 8)
                | (MACAddrL.at(2).digitValue() >> 16) | (MACAddrL.at(3).digitValue() >> 24);
        reg->setValueNumb(MACLow);
        break;
    }
    case REG_NETWORK_INTERFACE_CAPABILITIES:

        break;
    case REG_NETWORK_INTERFACE_CONF:

        break;
    case REG_CURRENT_IP_ADD:
    {
        QNetworkAddressEntry host = netInterface.addressEntries().at(interfaceNumber);
        int ip = host.ip().toIPv4Address();
        reg->setValueNumb(ip);
        break;
    }
    case REG_CURRENT_SUBNET_MASK:
        reg->setValueNumb((int) netInterface.addressEntries().at(0).netmask().toIPv4Address());
        break;
    case REG_CURRENT_DEFAULT_GATEWAY:
        //TODO: is platform dependent...
        reg->setValueNumb(0);
        break;
    case REG_PERSISTENT_IP_ADD:
        //TODO: need implements persistence
        reg->setValueNumb(0);
        break;
    case REG_PERSISTENT_SUBNET_MASK:
        //TODO: need implements persistence
        reg->setValueNumb(0);
        break;
    case REG_PERSISTENT_DEFAULT_GATEWAY:
        //TODO: need implements persistence
        reg->setValueNumb(0);
        break;
    case REG_LINK_SPEED:
        //TODO:
        reg->setValueNumb(0);
        break;
    default:
        break;
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

QNetworkInterface NetworkInterfaceRegisters::getInterfaceListWithoutLoopBack(int interfaceNumber)
{
    QList<QNetworkInterface> validInterfaces;
    foreach (QNetworkInterface interface, QNetworkInterface::allInterfaces()) {
        if(!(bool)(interface.flags() & QNetworkInterface::IsLoopBack)
                && interface.hardwareAddress().compare("00:00:00:00:00:00")!=0)
            validInterfaces.push_back(interface);
    }
    return validInterfaces.at(interfaceNumber);
}

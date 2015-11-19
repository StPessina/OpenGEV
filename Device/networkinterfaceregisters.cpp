#include "networkinterfaceregisters.h"

NetworkInterfaceRegisters::NetworkInterfaceRegisters(QNetworkInterface netInterface, int interfaceNumber)
{
    this->netInterface = netInterface;
    this->interfaceNumber = interfaceNumber;
    initRegisterMap();
    updateNetworkStatus();

    BootstrapRegister* networkCapabilitieReg = registers.at(
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
    BootstrapRegister* reg = registers.at(
                DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, offsetRegisterCode));
    return reg;
}

BootstrapRegister *NetworkInterfaceRegisters::getRegisterByAbsoluteRegCode(int regCode)
{
    int regType = DeviceRegisterConverter::getInterfaceNumberFromNetworkRegister(regCode);
    return getRegister(regType);
}

Status NetworkInterfaceRegisters::setRegister(int registerCode, int value, QHostAddress senderAddr, quint16 senderPort)
{
    BootstrapRegister* reg = getRegisterByAbsoluteRegCode(registerCode);
    if(reg==NULL) //CR-175cd
        return GEV_STATUS_INVALID_ADDRESS;

    int access = reg->getAccessType();

    if(access==RegisterAccess::RA_READ) //CR-175cd
        return GEV_STATUS_ACCESS_DENIED;

    reg->setValue(value);

    return GEV_STATUS_SUCCESS;
}

int NetworkInterfaceRegisters::getInterfaceNumber()
{
    return interfaceNumber;
}

void NetworkInterfaceRegisters::initRegisterMap()
{
    deviceMACAddressHighRegCode=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_DEVICE_MAC_ADD_HIGH);
    deviceMACAddessLowRegCode=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_DEVICE_MAC_ADD_LOW);
    networkInterfaceCapability=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_NETWORK_INTERFACE_CAPABILITIES);
    networkInterfaceConfiguration=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_NETWORK_INTERFACE_CONF);
    currentIPAddressRegCode=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_CURRENT_IP_ADD);
    currentSubnetMaskRegCode=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_CURRENT_SUBNET_MASK);
    currentDefaultGatewayRegCode=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_CURRENT_DEFAULT_GATEWAY);
    persintentIPAddressRegCode=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_PERSISTENT_IP_ADD);
    persistentSubnetMaskRegCode=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_PERSISTENT_SUBNET_MASK);
    persistentDefaultGatewayRegCode=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_PERSISTENT_DEFAULT_GATEWAY);
    linkSpeedRegCode=DeviceRegisterConverter::getNetworkInterfaceRegister(interfaceNumber, REG_LINK_SPEED);

    registers[deviceMACAddressHighRegCode]
            = new  BootstrapRegister(deviceMACAddressHighRegCode, "Device MAC Address - High (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ, 4);
    registers[deviceMACAddessLowRegCode]
     = new  BootstrapRegister(deviceMACAddessLowRegCode, "Device MAC Address - Low (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ, 4);
    registers[networkInterfaceCapability]
     = new  BootstrapRegister(networkInterfaceCapability, "Network Interface Capability (Network interface #" +
                                     to_string(interfaceNumber) + ")", RA_READ, 4);
    registers[networkInterfaceConfiguration]
     = new  BootstrapRegister(networkInterfaceConfiguration, "Network Interface Configuration (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ_WRITE, 4);
    registers[currentIPAddressRegCode]
     = new  BootstrapRegister(currentIPAddressRegCode, "Current IP Address (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ, 4);
    registers[currentSubnetMaskRegCode]
     = new  BootstrapRegister(currentSubnetMaskRegCode, "Current Subnet Mask (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ, 4);
    registers[currentDefaultGatewayRegCode]
     = new  BootstrapRegister(currentDefaultGatewayRegCode, "Current Default Gateway (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ, 4);
    registers[persintentIPAddressRegCode]
     = new  BootstrapRegister(persintentIPAddressRegCode, "Persistent IP Address (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ_WRITE, 4);
    registers[persistentSubnetMaskRegCode]
     = new  BootstrapRegister(persistentSubnetMaskRegCode, "Persistent Subnet Mask (Network interface #" +
                                     to_string(interfaceNumber) + ")",RA_READ_WRITE, 4);
    registers[persistentDefaultGatewayRegCode]
     = new  BootstrapRegister(persistentDefaultGatewayRegCode, "Persistent Default Gateway (Network interface #" +
                                     to_string(interfaceNumber) + ")", RA_READ_WRITE, 4);
    registers[linkSpeedRegCode]
     = new  BootstrapRegister(linkSpeedRegCode, "Link Speed (Network interface #" +
                              to_string(interfaceNumber) + ")",RA_READ, 4);
}

void NetworkInterfaceRegisters::updateNetworkStatus()
{
    QString MACAddr = netInterface.hardwareAddress();
    bool* ok = new bool;
    int MACHigh1 = MACAddr.section(':',1,1).toInt(ok,16);
    int MACHigh2 = MACAddr.section(':',0,0).toInt(ok,16);
    registers.at(deviceMACAddressHighRegCode)
            ->setValue((MACHigh1 | (MACHigh2 << 8))); //R-431cd

    int MACLow1 = MACAddr.section(':',5,5).toInt(ok,16);
    int MACLow2 = MACAddr.section(':',4,4).toInt(ok,16);
    int MACLow3 = MACAddr.section(':',3,3).toInt(ok,16);
    int MACLow4 = MACAddr.section(':',2,2).toInt(ok,16);
    registers.at(deviceMACAddessLowRegCode)
            ->setValue(MACLow1 | (MACLow2 << 8) | (MACLow3 << 16) | (MACLow4 << 24));
    delete ok;

    int ip = 0;
    if(netInterface.addressEntries().size()>0) {
        QNetworkAddressEntry host = netInterface.addressEntries().at(0);
        ip = host.ip().toIPv4Address();
    }
    registers.at(currentIPAddressRegCode)
            ->setValue(ip);

    int subnetMask = 0;
    if(netInterface.addressEntries().size()>0)
        subnetMask = (int) netInterface.addressEntries().at(0).netmask().toIPv4Address();
    registers.at(currentSubnetMaskRegCode)
            ->setValue(subnetMask);

    registers.at(currentDefaultGatewayRegCode)
            ->setValue(0);

    //Persistent IP CR-447cd
    registers.at(persintentIPAddressRegCode)
            ->setValue(0);

    //Persistent subnet mask CR-448cd
    registers.at(persistentSubnetMaskRegCode)
            ->setValue(0);

    //Persistent default gateway CR-449cd
    registers.at(persistentDefaultGatewayRegCode)
            ->setValue(0);

    //Link speed CR-450cd
    registers.at(linkSpeedRegCode)
            ->setValue(0);
}

#include "deviceregisters.h"


DeviceRegisterConverter::DeviceRegisterConverter()
{

}

int DeviceRegisterConverter::getNetworkInterfaceRegister(int interfaceNumber, int regType)
{
    switch (interfaceNumber) {
    case 0:
        switch (regType) {
        case REG_DEVICE_MAC_ADD_HIGH: return 0x0008;
        case REG_DEVICE_MAC_ADD_LOW: return 0x000C;
        case REG_NETWORK_INTERFACE_CAPABILITIES: return 0x0010;
        case REG_NETWORK_INTERFACE_CONF: return 0x0014;
        case REG_CURRENT_IP_ADD: return 0x0024;
        case REG_CURRENT_SUBNET_MASK: return 0x0034;
        case REG_CURRENT_DEFAULT_GATEWAY: return 0x0044;
        case REG_PERSISTENT_IP_ADD: return 0x064C;
        case REG_PERSISTENT_SUBNET_MASK: return 0x065C;
        case REG_PERSISTENT_DEFAULT_GATEWAY: return  0x066C;
        case REG_LINK_SPEED: return 0x0670;
        default:
            break;
        }
    case 1:
        return regType + 0x680;
    case 2:
        return regType + 0x700;
    case 3:
        return regType + 0x780;
    default:
        break;
    }
    return 0;
}

int DeviceRegisterConverter::getInterfaceNumberFromNetworkRegister(int regCode)
{
    if(regCode<=0x0670)
        return 0;
    if(regCode>=0x0680 && regCode<0x700)
        return 1;
    if(regCode>=0x0700 && regCode<0x780)
        return 2;
    if(regCode>=0x0780)
        return 3;
    return 0;
}

int DeviceRegisterConverter::getRegTypeFromNetworkRegister(int regCode)
{
    if(regCode<=0x0670) {
        switch (regCode) {
        case 0x0008: return REG_DEVICE_MAC_ADD_HIGH;
        case 0x000C: return REG_DEVICE_MAC_ADD_LOW;
        case 0x0010: return REG_NETWORK_INTERFACE_CAPABILITIES;
        case 0x0014: return REG_NETWORK_INTERFACE_CONF;
        case 0x0024: return REG_CURRENT_IP_ADD;
        case 0x0034: return REG_CURRENT_SUBNET_MASK;
        case 0x0044: return REG_CURRENT_DEFAULT_GATEWAY;
        case 0x064C: return REG_PERSISTENT_IP_ADD;
        case 0x065C: return REG_PERSISTENT_SUBNET_MASK;
        case 0x066C: return  REG_PERSISTENT_DEFAULT_GATEWAY;
        case 0x0670: return REG_LINK_SPEED;
        default:
            break;
        }
    }
    if(regCode>=0x0680 && regCode<0x700)
        return regCode - 0x0680;
    if(regCode>=0x0700 && regCode<0x780)
        return regCode - 0x0700;
    if(regCode>=0x0780)
        return regCode - 0x0780;
    return -1;
}

int DeviceRegisterConverter::getStreamChannelRegister(int streamChannelId, int regType)
{
    return 0x0D00 + streamChannelId*0x40 + regType;
}

int DeviceRegisterConverter::getChannelNumberFromStreamChannel(int regCode)
{
    return (regCode-0x0D00) % 0x40;
}

int DeviceRegisterConverter::getRegTypeFromStreamChannel(int regCode)
{
    return regCode - 0x0D00 - ((regCode-0x0D00) % 0x40)*0x40;
}

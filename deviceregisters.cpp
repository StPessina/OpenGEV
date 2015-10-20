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

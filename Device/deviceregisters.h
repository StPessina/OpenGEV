#ifndef DEVICEREGISTERS_H
#define DEVICEREGISTERS_H

#define REG_VESION 0x0000
#define REG_DEVICE_MODE 0x0004

#define REG_MANUFACTURE_NAME 0x0048
#define REG_MODEL_NAME 0x0068
#define REG_DEVICE_VERSION 0x0088
#define REG_MANUFACTURE_INFO 0x00A8
#define REG_SERIAL_NUMBER 0x00D8
#define REG_USER_DEFINIED_NAME 0x00E8
#define REG_FIRST_URL 0x0200
#define REG_SECOND_URL 0x0400

#define REG_NR_NETWORK_INTERFACE 0x0600

#define REG_NR_MESSAGE_CHANNELS 0x0900
#define REG_NR_STREAM_CHANNELS 0x0904
#define REG_ACTION_DEVICE_KEY 0x090C
#define REG_NR_ACTIVE_LINKS 0x0910
#define REG_GVSP_CAPABILITY 0x092C
#define REG_MESSAGE_CHANNEL_CAPABILITY 0x0930
#define REG_GVCP_CAPABILITY 0x0934
#define REG_HEARTBEAT_TIMEOUT 0x0938
#define REG_TIMEOUT_TICK_FREQUENCY_HIGH 0x093C
#define REG_TIMEOUT_TICK_FREQUENCY_LOW 0x0940
#define REG_TIMEOUT_CONTROL 0x0944
#define REG_DISCOVERY_ACK_DELAY 0x0950
#define REG_GVCP_CONFIGURATION 0x0954
#define REG_PENDING_TIMEOUT 0x0958
#define REG_CONTROL_SWITCHOVER_KEY 0x095C

#define REG_DEVICE_MAC_ADD_HIGH 0x0000
#define REG_DEVICE_MAC_ADD_LOW 0x0004
#define REG_NETWORK_INTERFACE_CAPABILITIES 0x0008
#define REG_NETWORK_INTERFACE_CONF 0x000C
#define REG_CURRENT_IP_ADD 0x001C
#define REG_CURRENT_SUBNET_MASK 0x002C
#define REG_CURRENT_DEFAULT_GATEWAY 0x003C
#define REG_PERSISTENT_IP_ADD 0x004C
#define REG_PERSISTENT_SUBNET_MASK 0x005C
#define REG_PERSISTENT_DEFAULT_GATEWAY 0x006C
#define REG_LINK_SPEED 0x0070

/**
 * @brief The DeviceRegisterConverter class utils conversion for device bootstrap register
 */
class DeviceRegisterConverter {
public:
    DeviceRegisterConverter();

    /**
     * @brief getNetworkInterfaceRegister return register for netword interface
     * @param interfaceNumber
     * @param regType relative register type
     * @return absolute register type
     */
    static int getNetworkInterfaceRegister(int interfaceNumber, int regType);

};

#endif // DEVICEREGISTERS_H

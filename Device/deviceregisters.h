#ifndef DEVICEREGISTERS_H
#define DEVICEREGISTERS_H

#define REG_VERSION 0x0000 //R-429cd
#define REG_DEVICE_MODE 0x0004 //R-430cd

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
#define REG_NR_ACTION_SIGNAL_CHANNELS 0x0908
#define REG_ACTION_DEVICE_KEY 0x090C
#define REG_NR_ACTIVE_LINKS 0x0910
#define REG_GVSP_CAPABILITY 0x092C //R-456cd
#define REG_MESSAGE_CHANNEL_CAPABILITY 0x0930 //R-457cd
#define REG_GVCP_CAPABILITY 0x0934 //R-458cd
#define REG_HEARTBEAT_TIMEOUT 0x0938 //R-459cd
#define REG_TIMEOUT_TICK_FREQUENCY_HIGH 0x093C
#define REG_TIMEOUT_TICK_FREQUENCY_LOW 0x0940
#define REG_TIMEOUT_CONTROL 0x0944
#define REG_DISCOVERY_ACK_DELAY 0x0950
#define REG_GVCP_CONFIGURATION 0x0954
#define REG_PENDING_TIMEOUT 0x0958 //R-472cd
#define REG_CONTROL_SWITCHOVER_KEY 0x095C

#define REG_CONTROL_CHANNEL_PRIVILEGE 0x0A00
#define REG_PRIMARY_APPLICATION_PORT 0x0A04
#define REG_PRIMARY_APPLICATION_IP_ADDRESS 0x0A14

#define REG_DEVICE_MAC_ADD_HIGH 0x0000 //R-431cd
#define REG_DEVICE_MAC_ADD_LOW 0x0004
#define REG_NETWORK_INTERFACE_CAPABILITIES 0x0008 //R-432cd
#define REG_NETWORK_INTERFACE_CONF 0x000C
#define REG_CURRENT_IP_ADD 0x001C
#define REG_CURRENT_SUBNET_MASK 0x002C
#define REG_CURRENT_DEFAULT_GATEWAY 0x003C
#define REG_PERSISTENT_IP_ADD 0x004C
#define REG_PERSISTENT_SUBNET_MASK 0x005C
#define REG_PERSISTENT_DEFAULT_GATEWAY 0x006C
#define REG_LINK_SPEED 0x0070

#define REG_STREAM_CHANNEL_PORT 0x0000
#define REG_STREAM_CHANNEL_PACKET_SIZE 0x0004
#define REG_STREAM_CHANNEL_PACKET_DELAY 0x0008
#define REG_STREAM_CHANNEL_PACKET_DESTINATION_ADDRESS 0x0018
#define REG_STREAM_CHANNEL_SOURCE_PORT 0x001C
#define REG_STREAM_CHANNEL_CAPABILITY 0x0020
#define REG_STREAM_CHANNEL_CONFIGURATION 0x0024
#define REG_STREAM_CHANNEL_ZONE 0x0028
#define REG_STREAM_CHANNEL_ZONE_DIRECTION 0x002C


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

    static int getInterfaceNumberFromNetworkRegister(int regCode);

    static int getRegTypeFromNetworkRegister(int regCode);

    static int getStreamChannelRegister(int streamChannelId, int regType);

    static int getChannelNumberFromStreamChannel(int regCode);

    static int getRegTypeFromStreamChannel(int regCode);

};

#endif // DEVICEREGISTERS_H

#include "devicestreamchannel.h"

DeviceStreamChannel::DeviceStreamChannel(int id)
{
    this->id = id;
    initRegisterMap();
}

DeviceStreamChannel::~DeviceStreamChannel()
{
    /*
    foreach(auto reg, registers)
        delete reg.second;
    */
}

BootstrapRegister *DeviceStreamChannel::getRegister(int regType)
{
    int regCode = DeviceRegisterConverter::getStreamChannelRegister(id, regType);

    return registers[regCode];
}

BootstrapRegister *DeviceStreamChannel::getRegisterByAbsoluteRegCode(int regCode)
{
    return registers[regCode];
}

void DeviceStreamChannel::initRegisterMap()
{
    int channelPort=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_PORT);
    int packetSize=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_PACKET_SIZE);
    int packetDelay=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_PACKET_DELAY);
    int packetDestinationAddress=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_PACKET_DESTINATION_ADDRESS);
    int sourcePort=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_SOURCE_PORT);
    int channelCapability=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_CAPABILITY);
    int channelConfiguration=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_CONFIGURATION);
    int channelZone=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_ZONE);
    int channelZoneDirection=DeviceRegisterConverter::getStreamChannelRegister(id, REG_STREAM_CHANNEL_ZONE_DIRECTION);

    registers[channelPort]
            = new  BootstrapRegister(channelPort, "Stream channel port (Stream channel #" +
                                     to_string(id) + ")",RA_READ_WRITE, 4);
    registers[packetSize]
            = new  BootstrapRegister(packetSize, "Stream channel packet size (Stream channel #" +
                                     to_string(id) + ")",RA_READ_WRITE, 4);
    registers[packetDelay]
            = new  BootstrapRegister(packetDelay, "Stream channel packet delay (Stream channel #" +
                                     to_string(id) + ")",RA_READ_WRITE, 4);
    registers[packetDestinationAddress]
            = new  BootstrapRegister(packetDestinationAddress, "Stream channel destination address (Stream channel #" +
                                     to_string(id) + ")",RA_READ_WRITE, 4);
    registers[sourcePort]
            = new  BootstrapRegister(sourcePort, "Stream channel source port (Stream channel #" +
                                     to_string(id) + ")",RA_READ, 4);
    registers[channelCapability]
            = new  BootstrapRegister(channelCapability, "Stream channel capability (Stream channel #" +
                                     to_string(id) + ")",RA_READ, 4);
    registers[channelConfiguration]
            = new  BootstrapRegister(channelConfiguration, "Stream channel configuration (Stream channel #" +
                                     to_string(id) + ")",RA_READ_WRITE, 4);
    registers[channelZone]
            = new  BootstrapRegister(channelZone, "Stream channel zone (Stream channel #" +
                                     to_string(id) + ")",RA_READ, 4);
    registers[channelZoneDirection]
            = new  BootstrapRegister(channelZoneDirection, "Stream channel zone direction (Stream channel #" +
                                     to_string(id) + ")",RA_READ, 4);
}

#include "streamchanneltransmitter.h"

StreamChannelTransmitter::StreamChannelTransmitter(int id)
{
    this->id = id;
    initRegisterMap();

    int sourcePort = rand()*10000+40000; //select random port between rangeport 40000-50000

    registers[DeviceRegisterConverter::getStreamChannelRegister(id,REG_STREAM_CHANNEL_SOURCE_PORT)]
            ->setValueNumb(sourcePort);
}

StreamChannelTransmitter::StreamChannelTransmitter(int id, int sourcePort)
{
    this->id = id;
    initRegisterMap();

    registers[DeviceRegisterConverter::getStreamChannelRegister(id,REG_STREAM_CHANNEL_SOURCE_PORT)]
            ->setValueNumb(sourcePort);
}

StreamChannelTransmitter::~StreamChannelTransmitter()
{
    if(isChannelOpen())
        closeStreamChannel();

    foreach(auto reg, registers)
        delete reg.second;
}

BootstrapRegister *StreamChannelTransmitter::getRegister(int regType)
{
    int regCode = DeviceRegisterConverter::getStreamChannelRegister(id, regType);

    return registers[regCode];
}

BootstrapRegister *StreamChannelTransmitter::getRegisterByAbsoluteRegCode(int regCode)
{
    return registers[regCode];
}

void StreamChannelTransmitter::openStreamChannel(QHostAddress destAddress, quint16 destPort)
{
    closeStreamChannel(); //Check port different from 0 (R-087ca)

    BootstrapRegister* gvspSCPD =
            registers[DeviceRegisterConverter::getStreamChannelRegister(id,REG_STREAM_CHANNEL_SOURCE_PORT)];

    streamChannelTransmitter = new UDPChannelTransmitter(destAddress, gvspSCPD->getValueNumb());
    streamChannelTransmitter->initSocket();

    BootstrapRegister* gvspSCP = registers[DeviceRegisterConverter::getStreamChannelRegister(id,REG_STREAM_CHANNEL_PORT)];

    gvspSCP->setValueNumb(
                (gvspSCP->getValueNumb() & (destPort & 0x00FF)) //set new destination port value
                );
    this->destPort = destPort;

    BootstrapRegister* gvspSCDA =
            registers[DeviceRegisterConverter::getStreamChannelRegister(id,REG_STREAM_CHANNEL_PACKET_DESTINATION_ADDRESS)];
    gvspSCDA->setValueNumb((int) destAddress.toIPv4Address());
    this->destAddress = destAddress;
}

void StreamChannelTransmitter::closeStreamChannel()
{
    if(isChannelOpen()) {
        destPort = 0;
        BootstrapRegister* gvspSCP = registers[DeviceRegisterConverter::getStreamChannelRegister(id,REG_STREAM_CHANNEL_PORT)];
        gvspSCP->setValueNumb(gvspSCP->getValueNumb() & 0xFF00); //Reset port


        destAddress.clear();
        BootstrapRegister* gvspSCDA =
                registers[DeviceRegisterConverter::getStreamChannelRegister(id,REG_STREAM_CHANNEL_PACKET_DESTINATION_ADDRESS)];
        gvspSCDA->setValueNumb(0);

        delete streamChannelTransmitter;
    }
}

bool StreamChannelTransmitter::isChannelOpen()
{
    BootstrapRegister* gvspSCP = registers[DeviceRegisterConverter::getStreamChannelRegister(id,REG_STREAM_CHANNEL_PORT)];

    if((gvspSCP->getValueNumb() & 0xFF) != 0)
        return true;
    return false;
}

void StreamChannelTransmitter::initRegisterMap()
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
